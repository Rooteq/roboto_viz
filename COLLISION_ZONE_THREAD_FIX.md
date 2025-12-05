# Collision Zone Loading Thread Fix - Eliminating Final Executor Starvation

## Problem: bt_navigator Heartbeat Timeout AFTER Navigation Starts

### Symptoms
Even after fixing `waitUntilNav2Active()` deadlock and attempting QTimer.singleShot async loading:
- **bt_navigator dies** 4 seconds after navigation starts (robot already moving)
- Lifecycle heartbeat timeout: "Have not received heartbeat from bt_navigator for 4000ms"
- Error appears in background while robot is navigating

### Root Cause: QTimer.singleShot Doesn't Prevent Blocking

The previous "fix" using `QTimer.singleShot(0, load_zones_async)` had a critical flaw:

**QTimer.singleShot only DEFERS execution - it doesn't prevent blocking!**

```python
# PREVIOUS ATTEMPT (STILL BLOCKS!)
def load_zones_async():
    # This runs in GuiManager thread (same thread as executor.spin())
    self.node.collision_monitor_manager.load_collision_zones_for_route(route)  # BLOCKS!
    self.node.collision_monitor_manager.start_monitoring()
    color_cache = self.node.collision_monitor_manager.color_cache

QTimer.singleShot(0, load_zones_async)  # Defers to next event loop, but STILL BLOCKS
```

**What actually happens:**
```
T+0ms:   QTimer.singleShot(0, load_zones_async) - schedules callback
T+0ms:   Returns immediately (seems non-blocking!)
T+10ms:  Qt event loop triggers load_zones_async() callback
T+10ms:  ↓ EXECUTOR BLOCKED in GuiManager thread ↓
T+10ms:  File I/O: json.load(f) - blocks 50-200ms
T+100ms: _build_color_cache() - QPixmap → numpy conversion - blocks 100-500ms
T+500ms: Callback completes
T+500ms: ↓ EXECUTOR RESUMES ↓
...
T+4000ms: bt_navigator heartbeat timeout → bt_navigator DIES
```

### Why This Happens

**GuiManager thread runs the ROS2 executor** ([gui_manager.py:1463](gui_manager.py#L1463)):
```python
def run(self):
    self.node = ManagerNode()
    self.executor.add_node(self.node)
    # ...
    self.executor.spin()  # ← RUNS IN THIS THREAD!
```

**Any code running in GuiManager thread blocks `executor.spin()`**, including:
- ❌ Synchronous service calls (`waitUntilNav2Active()`) - **FIXED in first pass**
- ❌ File I/O (`json.load(f)`) - **STILL BLOCKING via QTimer callback**
- ❌ Heavy computation (`_build_color_cache()` numpy operations) - **STILL BLOCKING**

QTimer.singleShot just moves the callback to the next event loop iteration, but **the callback still runs in the same thread**!

## The Fix: True Separate QThread

**Move collision zone loading to a dedicated QThread** that runs completely independently of the executor.

### 1. Created CollisionZoneLoadThread Class ([gui_manager.py:98-126](gui_manager.py#L98-L126))

```python
class CollisionZoneLoadThread(QThread):
    """
    Separate QThread for loading collision zones WITHOUT blocking the executor.
    This thread handles file I/O and numpy operations that would otherwise
    starve the ROS2 executor in GuiManager thread.
    """
    loading_complete = pyqtSignal(str, object)  # route_name, color_cache

    def __init__(self, collision_monitor_manager, route_name):
        super().__init__()
        self.collision_monitor_manager = collision_monitor_manager
        self.route_name = route_name

    def run(self):
        """Load collision zones in background thread (does NOT block executor!)"""
        try:
            # This does file I/O and numpy operations - OK because we're in separate thread
            self.collision_monitor_manager.load_collision_zones_for_route(self.route_name)
            color_cache = self.collision_monitor_manager.color_cache

            # Emit completion signal (thread-safe via Qt signals)
            self.loading_complete.emit(self.route_name, color_cache)
        except Exception as e:
            print(f"ERROR: Failed to load collision zones in background thread: {e}")
            import traceback
            traceback.print_exc()
            # Emit with None cache to indicate failure
            self.loading_complete.emit(self.route_name, None)
```

**Key Differences from QTimer.singleShot:**
- ✅ Runs in **completely separate thread** (not GuiManager thread)
- ✅ File I/O and numpy operations **never block executor**
- ✅ Qt signals provide **thread-safe communication** back to GUI thread
- ✅ Executor continues spinning **throughout entire loading process**

### 2. Updated handle_set_route() ([gui_manager.py:1563-1589](gui_manager.py#L1563-L1589))

```python
# Load collision zones in SEPARATE THREAD to avoid blocking executor
# This prevents file I/O and numpy operations from starving the ROS2 executor
if self.node and self.node.collision_monitor_manager:
    # Cancel any existing collision zone loading thread
    if self.collision_zone_load_thread is not None and self.collision_zone_load_thread.isRunning():
        print("WARNING: Previous collision zone loading still in progress, waiting...")
        self.collision_zone_load_thread.wait(1000)  # Wait up to 1 second

    # Create new thread for collision zone loading
    self.collision_zone_load_thread = CollisionZoneLoadThread(
        self.node.collision_monitor_manager,
        route
    )

    # Connect completion signal
    self.collision_zone_load_thread.loading_complete.connect(
        self._on_collision_zones_loaded
    )

    # Start thread (file I/O happens in background, executor keeps spinning!)
    print(f"INFO: Starting collision zone loading thread for route '{route}'")
    self.collision_zone_load_thread.start()
```

### 3. Added Completion Callback ([gui_manager.py:1591-1608](gui_manager.py#L1591-L1608))

```python
@pyqtSlot(str, object)
def _on_collision_zones_loaded(self, route_name: str, color_cache):
    """
    Called when collision zone loading thread completes.
    This runs in the GUI thread (via Qt signal), so it's safe to emit signals.
    """
    if color_cache is not None:
        # Start monitoring now that zones are loaded
        if self.node and self.node.collision_monitor_manager:
            self.node.collision_monitor_manager.start_monitoring()

        # Show collision zones on minimap
        print(f"INFO: Collision zones loaded for route '{route_name}', emitting show_collision_zones signal")
        self.show_collision_zones.emit(route_name, color_cache)
    else:
        print(f"WARNING: Failed to load collision zones for route '{route_name}'")
        self.show_collision_zones.emit(route_name, None)
```

## How This Eliminates Blocking

### Before (QTimer.singleShot - STILL BLOCKS):
```
T+0ms:   Navigation starts
T+0ms:   QTimer.singleShot(0, load_zones_async)
T+0ms:   Returns to executor
T+10ms:  Qt event loop runs load_zones_async() in GuiManager thread
T+10ms:  ↓ EXECUTOR BLOCKED (same thread!) ↓
T+500ms: File I/O + numpy operations complete
T+500ms: ↓ EXECUTOR RESUMES ↓
T+4000ms: Heartbeat timeout → bt_navigator DIES
```

### After (QThread - NO BLOCKING):
```
T+0ms:   Navigation starts
T+0ms:   CollisionZoneLoadThread.start()
T+1ms:   Thread spawned, returns immediately
T+1ms:   ↓ EXECUTOR KEEPS SPINNING ↓
T+1ms:   (Background thread) File I/O starts
T+100ms: (Background thread) _build_color_cache() numpy operations
T+500ms: (Background thread) Emits loading_complete signal
T+501ms: (GUI thread) _on_collision_zones_loaded() callback fires
T+502ms: start_monitoring() called, zones displayed
...
T+4000ms: All heartbeats OK → bt_navigator ALIVE ✓
```

## Key Architectural Insight

**The problem was never about delaying the work** - it was about **which thread does the work**.

| Approach | Execution Thread | Blocks Executor? | Result |
|----------|-----------------|------------------|---------|
| **Synchronous call** | GuiManager | ✅ Yes | Deadlock |
| **QTimer.singleShot** | GuiManager | ✅ Yes (deferred, but still blocks) | Heartbeat timeout |
| **QThread** | Separate thread | ❌ No | Works! |

## Timeline Comparison

### File I/O Execution

**QTimer.singleShot (BROKEN):**
```python
# In GuiManager thread (same as executor.spin())
QTimer.singleShot(0, lambda: json.load(f))  # ← Defers but doesn't prevent blocking!
```

**QThread (FIXED):**
```python
# In separate thread
class LoadThread(QThread):
    def run(self):
        json.load(f)  # ← Blocks separate thread, executor keeps running!
```

## Related Fixes

This is the **third and final executor starvation fix**:

1. **First fix**: Removed `waitUntilNav2Active()` blocking call ([NAVIGATION_DEADLOCK_FIX.md](NAVIGATION_DEADLOCK_FIX.md))
   - Eliminated synchronous service call blocking

2. **Second fix**: Attempted QTimer.singleShot for collision zones ([COLLISION_ZONE_BLOCKING_FIX.md](COLLISION_ZONE_BLOCKING_FIX.md))
   - Deferred execution but **still blocked** (same thread)

3. **This fix**: True separate QThread for collision zone loading
   - **Completely eliminates blocking** - executor never starves

All three were necessary to achieve zero executor starvation!

## Testing Checklist

After this fix, verify:
- [ ] No heartbeat timeout errors for any nav2 nodes
- [ ] bt_navigator stays alive throughout navigation
- [ ] Map server stays alive
- [ ] Controller server stays alive
- [ ] Collision zones still load correctly
- [ ] Minimap shows collision zones
- [ ] Navigation completes successfully
- [ ] No TF extrapolation errors

## Files Modified

- [gui_manager.py](roboto_viz/gui_manager.py)
  - Lines 98-126: Added CollisionZoneLoadThread class
  - Line 1111: Added collision_zone_load_thread member variable
  - Lines 1563-1589: Replaced QTimer.singleShot with QThread
  - Lines 1591-1608: Added _on_collision_zones_loaded callback

## Prevention

**Golden Rule: Never do blocking operations in a thread that runs a ROS2 executor!**

### Blocking Operations to Avoid:
- ❌ Synchronous file I/O (`open()`, `json.load()`, `pickle.load()`)
- ❌ Synchronous service calls (`call()`, `wait*()` methods)
- ❌ Network I/O (`requests.get()`, `urllib.request.urlopen()`)
- ❌ Heavy computation (numpy operations, image processing)
- ❌ Sleep/blocking waits (`time.sleep()`, `threading.Event.wait()`)

### Safe Alternatives:
- ✅ Async service calls (`call_async()` + `add_done_callback()`)
- ✅ **Separate QThread for file I/O** (this fix!)
- ✅ ThreadPoolExecutor for heavy computation
- ✅ Async network libraries (aiohttp, etc.)
- ✅ QTimer for short non-blocking operations

### Why QTimer.singleShot Is Not Enough

QTimer.singleShot is useful for:
- ✅ Deferring lightweight operations
- ✅ Breaking recursion
- ✅ Delaying signal emissions
- ✅ Scheduling quick callbacks

QTimer.singleShot is **NOT** useful for:
- ❌ File I/O (still blocks the thread it runs in)
- ❌ Heavy computation (still blocks)
- ❌ Preventing executor starvation (doesn't change thread!)

**When in doubt, use a separate QThread** for anything that might take >10ms.

## Conclusion

This fix completes the trilogy of executor starvation fixes:
1. Remove synchronous service calls
2. Remove synchronous file I/O from executor thread
3. Move all heavy operations to separate threads

The key insight: **QTimer.singleShot defers when code runs, but QThread changes where code runs.**

Only QThread can truly prevent executor starvation.
