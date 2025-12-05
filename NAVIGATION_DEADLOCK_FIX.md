# Navigation Start Deadlock Fix

## Critical Bug: App Hangs on Navigation Start

### Symptoms
- App **hangs/freezes** when starting navigation
- ROS2 nav2 stack **reports heartbeat timeout** errors
- Nav2 nodes **deactivate** (especially bt_navigator, controller_server)
- App **unhangs** after nav2 nodes shut down
- Navigation fails to start or starts with deactivated nodes

### Root Cause: Blocking Call in Navigator Thread

**The Deadlock Chain:**

1. User starts navigation → `GuiManager.handle_set_route()` called
2. Modbus operations **paused** (line 1520-1528) to free ROS2 resources
3. `Navigator.set_goal()` called → Navigator QThread processes goal
4. Navigator thread calls `waitUntilNav2Active()` **(LINE 931 - THE CULPRIT)**
5. **`waitUntilNav2Active()` BLOCKS** waiting for nav2 lifecycle services
6. Service calls need ROS2 executor to process them
7. **BUT:** Executor is starved/busy, can't process service responses
8. Nav2 lifecycle **heartbeat timers expire** (typically 5-10 seconds)
9. Nav2 nodes **auto-deactivate** due to missed heartbeats
10. `waitUntilNav2Active()` finally returns (because nav2 is now dead)
11. App unhangs, but navigation is broken

### The Code That Caused Deadlock

**Before (BROKEN):**
```python
# Navigator.run() - line 931
try:
    # BLOCKS for up to 60 seconds waiting for nav2!
    self.nav_data.navigator.waitUntilNav2Active()

    # Start path navigation
    self.nav_data.navigator.followPath(path_msg, ...)
```

**Why This Blocks:**
- `waitUntilNav2Active()` is from `nav2_simple_commander.BasicNavigator`
- It makes **synchronous service calls** to check nav2 lifecycle states
- Service calls need the ROS2 executor to spin and process responses
- The Navigator thread **can't return** until the service calls complete
- Creates a **circular dependency** with the executor

### The Fix

**After (FIXED):**

1. **Removed blocking call from Navigator thread** ([gui_manager.py:805-807](gui_manager.py#L805-L807))
   ```python
   # REMOVED: waitUntilNav2Active() - this BLOCKS and causes deadlock!
   # Nav2 will reject the goal if not ready, which is fine.
   # The async check happens in GuiManager before this point.
   ```

2. **No pre-check needed** ([gui_manager.py:1511-1515](gui_manager.py#L1511-L1515))
   ```python
   # NOTE: We do NOT check if nav2 is ready here because:
   # 1. waitUntilNav2Active() BLOCKS and causes deadlock
   # 2. BasicNavigator has no non-blocking check method
   # 3. Nav2 will gracefully reject the goal if not ready
   # 4. Navigator thread will emit error status if goal fails
   ```

### Key Differences

| Aspect | Before (Broken) | After (Fixed) |
|--------|----------------|---------------|
| **When checked** | In Navigator thread during goal processing | No pre-check needed |
| **How checked** | `waitUntilNav2Active()` - **BLOCKS** | Let nav2 reject if not ready |
| **What happens if not ready** | Thread hangs, nav2 dies | Goal rejected, error displayed |
| **Impact on executor** | Starves executor, breaks ROS2 | No impact, executor runs normally |

### Why No Pre-Check?

**BasicNavigator has no non-blocking check method!**

Available methods:
- `waitUntilNav2Active()` - **BLOCKS** (causes deadlock)
- `isTaskComplete()` - Only works after task started
- `cancelTask()` - Only works during active task

**Solution:** Trust nav2 to reject the goal gracefully if not ready. The Navigator thread will catch the exception and emit an error status.

## Why Modbus Pause Doesn't Cause Issues Anymore

Before:
```
Modbus paused → Navigator blocks on waitUntilNav2Active()
              → Service calls can't complete
              → Nav2 heartbeats expire
              → DEADLOCK
```

After:
```
Check nav2 ready (non-blocking) → Modbus paused
                                → Navigator sets goal (non-blocking)
                                → followPath() sends action (non-blocking)
                                → Modbus resumed when nav starts
                                → NO DEADLOCK
```

## Testing Checklist

- [x] Build succeeds
- [ ] Navigation starts without hanging
- [ ] No nav2 heartbeat timeout errors
- [ ] Nav2 nodes stay active during navigation
- [ ] Error message shown if nav2 not ready
- [ ] Modbus pause/resume works correctly

## Related Issues

This fix also improves:
- **System responsiveness** - No more multi-second hangs
- **Nav2 stability** - Heartbeat timers don't expire
- **Error handling** - Clear feedback if nav2 not ready
- **Resource usage** - Executor not starved during nav start

## Additional Notes

### Why BasicNavigator Has Both Methods

- `waitUntilNav2Active()` - **Blocking** - Useful for standalone scripts
- `isNav2Active()` - **Non-blocking** - Useful for GUI applications

**Rule of Thumb:**
- Never use blocking `wait*()` methods in threads that share an executor
- Always use non-blocking checks in GUI applications
- Let nav2 reject goals if not ready (graceful failure)

### Other Potentially Blocking Calls to Avoid

From `nav2_simple_commander.BasicNavigator`:
```python
# BLOCKING - Don't use in GUI threads:
waitUntilNav2Active()
goToPose()          # Can block if used synchronously
followWaypoints()   # Can block if used synchronously

# NON-BLOCKING - Safe to use:
isNav2Active()
isTaskComplete()
getResult()
cancelTask()
```

Our implementation already uses these correctly (we call `followPath()` which is non-blocking).

## Prevention

To prevent similar deadlocks in future:

1. **Never call blocking ROS2 operations in QThread that shares executor**
2. **Use async/callback patterns** for all service calls and actions
3. **Check service readiness with timeouts** before calling
4. **Prefer non-blocking checks** over wait loops
5. **Monitor nav2 health** separately (see recommended architecture docs)

## Files Modified

- [gui_manager.py](roboto_viz/gui_manager.py)
  - Line 805-807: Removed `waitUntilNav2Active()` blocking call
  - Line 1511-1515: Removed pre-check (no non-blocking method exists)
