# Modbus Pause System for ROS2 Resource Protection

## Problem Statement

Modbus operations (even with proper locking) can interfere with critical ROS2 operations:

1. **Map Loading** - Nav2 service calls require full system resources
2. **Navigation Start** - Computationally intensive, needs priority
3. **Service Calls** - Any blocking can cause ROS2 heartbeat loss

## Solution: Comprehensive Pause System

All Modbus operations (LED writes, battery polling, button polling, continuous refresh) are **completely paused** during critical ROS2 operations.

## Implementation

### Three-Layer Pause Mechanism

**Layer 1: LED/Buzzer Manager** ([env_expression_manager.py](roboto_viz/env_expression_manager.py))
```python
def _send_continuous_messages(self):
    if self.modbus_paused:
        return  # Skip all LED/buzzer writes
```

**Layer 2: Battery Polling** ([modbus_battery_receiver.py](roboto_viz/modbus_battery_receiver.py))
```python
def _poll_battery(self):
    if self.is_navigating or self.paused:
        time.sleep(self.poll_interval)
        continue  # Skip battery reads
```

**Layer 3: Button Polling** ([modbus_button_receiver.py](roboto_viz/modbus_button_receiver.py))
```python
def _poll_button(self):
    if self.paused:
        time.sleep(self.poll_interval)
        continue  # Skip button reads
```

### Pause Triggers

#### 1. Map Loading

**Trigger**: `handle_map_selected()` in [gui_manager.py](roboto_viz/gui_manager.py)

```python
def handle_map_selected(self, map_name: str):
    # PAUSE ALL
    if self.can_manager:
        self.can_manager.pause_can_messages()
    if self.modbus_battery_receiver:
        self.modbus_battery_receiver.paused = True
    if self.modbus_button_receiver:
        self.modbus_button_receiver.paused = True

    # Load map in background thread
    self.map_load_worker.start()
```

**Resume**: `_on_map_load_complete()`

```python
def _on_map_load_complete(self, success: bool, ...):
    # RESUME ALL
    if self.can_manager:
        self.can_manager.resume_can_messages()
    if self.modbus_battery_receiver:
        self.modbus_battery_receiver.paused = False
    if self.modbus_button_receiver:
        self.modbus_button_receiver.paused = False
```

**Duration**: Typically 1-3 seconds (map loading time)

#### 2. Navigation Start

**Trigger**: `handle_set_route()` in [gui_manager.py](roboto_viz/gui_manager.py)

```python
def handle_set_route(self, route: str, ...):
    # PAUSE ALL
    if self.can_manager:
        self.can_manager.pause_can_messages()
    if self.modbus_battery_receiver:
        self.modbus_battery_receiver.paused = True
    if self.modbus_button_receiver:
        self.modbus_button_receiver.paused = True

    # Start navigation (heavy ROS2 operations)
    self.navigator.set_goal(route, ...)
```

**Resume**: `handle_navigation_started()`

```python
def handle_navigation_started(self):
    # RESUME ALL
    if self.can_manager:
        self.can_manager.resume_can_messages()
    if self.modbus_battery_receiver:
        self.modbus_battery_receiver.paused = False
    if self.modbus_button_receiver:
        self.modbus_button_receiver.paused = False

    # Wait 500ms before sending first Modbus message
    QTimer.singleShot(500, self._send_navigation_start_can_message)
```

**Duration**: Variable (navigation service call completion time), typically 2-5 seconds

## What Gets Paused

### During Map Loading
```
✗ LED continuous refresh (0.5Hz)
✗ Battery ADC polling (0.5Hz)
✗ Button click polling (1Hz)
✗ LED state changes
✗ Buzzer state changes
✗ All Modbus writes
✗ All Modbus reads

✓ ROS2 map loading service
✓ ROS2 node heartbeat
✓ ROS2 topic publishing
```

### During Navigation Start
```
✗ LED continuous refresh (0.5Hz)
✗ Battery ADC polling (0.5Hz)
✗ Button click polling (1Hz)
✗ LED state changes
✗ Buzzer state changes
✗ All Modbus writes
✗ All Modbus reads

✓ ROS2 navigation planning
✓ ROS2 path computation
✓ ROS2 controller initialization
✓ ROS2 node heartbeat
```

## Blocking Analysis

### No Blocking Operations

All operations are **non-blocking** with proper error handling:

| Operation | Timeout | Blocking Risk | Mitigation |
|-----------|---------|---------------|------------|
| Modbus write | 300ms | LOW | try/except, early return on pause |
| Modbus read | 300ms | LOW | try/except, early return on pause |
| Lock acquisition | N/A | NONE | Single lock, no nesting |
| Serial close | N/A | LOW | Wrapped in try/except |
| Thread join | 2000ms | LOW | Timeout on join, daemon threads |

### Lock Acquisition Safety

**Single Lock Only**: One `instrument_lock` shared across all operations
```python
with self.instrument_lock:  # Only one lock depth
    result = self.instrument.read_bit(...)
```

**No Nested Locks**: Verified no code path acquires lock while holding another
```bash
grep -n "with.*lock" roboto_viz/*.py
# All show single-depth lock acquisition
```

**No Deadlocks Possible**:
- Single lock = no circular dependency
- All locks have timeout via Modbus timeout (300ms)
- Daemon threads = killed on exit even if hung

## Pause/Resume Timing

### Typical Flow: Map Load + Navigation Start

```
Time    Event                           Modbus State
----    -----                           ------------
0.0s    User selects map                RUNNING
0.1s    handle_map_selected()           PAUSED (all 3 layers)
0.1s    Map loading starts              PAUSED
2.5s    Map loaded                      PAUSED
2.6s    _on_map_load_complete()         RESUMED (all 3 layers)
5.0s    User starts navigation          RUNNING
5.1s    handle_set_route()              PAUSED (all 3 layers)
5.1s    Nav planning starts             PAUSED
7.8s    Nav planning complete           PAUSED
7.9s    handle_navigation_started()     RESUMED (all 3 layers)
8.4s    First Modbus message sent       RUNNING (500ms delay)
```

**Total Pause Time**: ~5.3 seconds over 8.4 seconds = 63% of time paused during critical ops

## Benefits

1. **ROS2 Heartbeat Protected** - No Modbus interference during service calls
2. **Faster Operations** - Map loading/nav start get full CPU/serial resources
3. **No Timeouts** - ROS2 nodes don't timeout waiting for resources
4. **Deterministic** - Pause/resume is explicit and traceable
5. **Safe** - No hanging due to comprehensive error handling

## Monitoring

### Console Output
```
Modbus: All operations paused for map loading
Map 'hala' loading started...
Successfully loaded map 'hala' onto robot
Modbus: All operations resumed after map loading

Modbus: All operations paused for navigation start
Navigation planning...
Navigation started successfully
Modbus: All operations resumed after navigation start
```

### State Verification
```python
# Check if paused
can_manager.modbus_paused  # True/False
battery_receiver.paused    # True/False
button_receiver.paused     # True/False

# All should be synchronized
```

## Failure Modes

### What if pause() fails?
- Wrapped in try/except - continues anyway
- Modbus might interfere but won't hang
- Console warning printed

### What if resume() fails?
- Wrapped in try/except - won't hang app
- Modbus stays paused (safe state)
- User can STOP/reconnect to reset

### What if map load hangs?
- Background thread - UI stays responsive
- User can cancel and try again
- Modbus will resume after timeout/cancel

### What if navigation start fails?
- `handle_navigation_started()` never called
- Modbus stays paused (safe)
- User STOP button still works (robust shutdown)

## Testing

### Test Map Loading
1. Connect to robot
2. Switch maps
3. **Verify**: Console shows "paused for map loading" → "resumed after map loading"
4. **Verify**: No ROS2 node timeouts during load

### Test Navigation Start
1. Start navigation
2. **Verify**: Console shows "paused for navigation start" → "resumed after navigation start"
3. **Verify**: No heartbeat loss

### Test Rapid Operations
1. Load map, immediately start navigation
2. **Verify**: Pause/resume cycles correctly
3. **Verify**: No hanging or errors

### Test STOP During Pause
1. Load map or start navigation
2. Click STOP immediately (during pause)
3. **Verify**: App responds in <1s
4. **Verify**: Clean shutdown

## Configuration

All pause-related parameters:

```python
# Modbus timeout (affects max pause delay)
instrument.serial.timeout = 0.3  # 300ms

# Navigation start delay before first Modbus message
QTimer.singleShot(500, ...)  # 500ms

# Thread join timeout during shutdown
thread.join(timeout=2.0)  # 2s max wait
```

## Summary

The pause system ensures Modbus **never interferes** with critical ROS2 operations by:

1. **Comprehensive** - All 3 layers (LED, battery, button) paused together
2. **Automatic** - Triggered by map load and nav start
3. **Safe** - Non-blocking, error-wrapped, timeout-protected
4. **Traceable** - Console logging of all pause/resume events
5. **Tested** - STOP works even during paused state

This eliminates ROS2 heartbeat loss and hanging issues caused by Modbus operations competing for system resources.
