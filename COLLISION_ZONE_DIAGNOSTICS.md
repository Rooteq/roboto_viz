# Collision Zone Detection - Diagnostic Guide

## Enhanced Debug Messages

The code now includes extensive debug logging to help identify why zone detection isn't working.

## What to Look For When Running

### 1. Map Loading Phase

When you load a map, look for these messages:

```
DEBUG: GuiManager.handle_map_selected called with map: <mapname>
DEBUG: Created CollisionMonitorManager
DEBUG: CollisionMonitorManager.set_current_map: <mapname>
DEBUG: Loading collision zones for map: <mapname>
DEBUG: Collision map path: /home/user/.robotroutes/maps/collision_<mapname>.pgm
DEBUG: Collision zones JSON path: /home/user/.robotroutes/maps/collision_<mapname>_zones.json
```

**Expected**: Should show paths to collision files

**If you see**: "No collision mask found" or "No collision zones file found"
- **Problem**: Collision zones haven't been created/saved for this map
- **Solution**: Open plan editor → Edytuj Strefy Kolizji → Create zones → Paint → Save

### 2. Zone Loading Confirmation

```
DEBUG: Loaded collision mask: /home/user/.robotroutes/maps/collision_<mapname>.pgm
DEBUG: Mask dimensions: 384x384
DEBUG: Loaded 2 collision zones
DEBUG:   Zone 1: points=[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]], color=RGB(255, 100, 100)
DEBUG:   Zone 2: points=[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]], color=RGB(100, 255, 100)
```

**Expected**: Shows number of zones, their points, and colors

**If you see**: "Loaded 0 collision zones"
- **Problem**: No zones defined in JSON file
- **Solution**: Create zones in the collision zone editor

### 3. Monitoring Started

```
DEBUG: Started collision zone monitoring
DEBUG: Collision monitor manager started for map: <mapname>
```

**Expected**: Confirms monitoring is active

### 4. Position Updates

Every ~2.5 seconds (50 updates at 20Hz):

```
DEBUG: CollisionMonitor position update: (5.234, 3.456) [update #50]
DEBUG: CollisionMonitor position update: (5.240, 3.460) [update #100]
```

**Expected**: Position updates with incrementing counter

**If you DON'T see these**:
- **Problem**: Robot position not being sent to collision monitor
- **Possible causes**:
  - Robot not connected
  - Pose topic not publishing
  - emit_pose() not being called

### 5. Position Checking

Every ~10 seconds (50 checks at 5Hz timer):

```
DEBUG: Checking position - world=(5.234, 3.456), pixel=(523, 345), map_size=(384, 384)
DEBUG: Pixel at (523, 345) has color RGB(255, 255, 255)
DEBUG: Zone at position: None (current_zone: None)
```

**Expected**: Shows world coords, pixel coords, map size, pixel color, and zone detection

**If you see "CollisionMonitor not enabled"**:
- **Problem**: Monitoring wasn't started
- **Solution**: Check that start_monitoring() was called after map load

**If you see "Robot position not set yet"**:
- **Problem**: No position updates received
- **Solution**: Check robot connection and pose topic

**If you see "No collision mask pixmap loaded"**:
- **Problem**: Collision mask file not loaded
- **Solution**: Create and save collision zones for the map

**If you see "No collision zones loaded"**:
- **Problem**: JSON file exists but has no zones
- **Solution**: Create zones in the collision zone editor

### 6. When Robot is on a Painted Zone

```
DEBUG: Checking position - world=(5.234, 3.456), pixel=(523, 345), map_size=(384, 384)
DEBUG: Pixel at (523, 345) has color RGB(255, 100, 100)
DEBUG: Matched zone 1 with color RGB(255, 100, 100)
DEBUG: Zone at position: 1 (current_zone: None)
DEBUG: Zone transition detected - from zone None to zone 1
DEBUG: Robot position: world=(5.234, 3.456), pixel=(523, 345)
DEBUG: ===== ROBOT ENTERED COLLISION ZONE 1 =====
DEBUG: Setting collision polygon points to: [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
DEBUG: Executing ROS2 command: ros2 param set /collision_monitor PolygonSlow.points [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
DEBUG: ✓ Successfully set collision polygon to: [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
```

**Expected**: See color match, zone detection, transition, and ROS2 command

### 7. When Robot Exits Zone

```
DEBUG: Checking position - world=(8.123, 4.567), pixel=(812, 456)
DEBUG: Pixel at (812, 456) has color RGB(255, 255, 255)
DEBUG: Zone at position: None (current_zone: 1)
DEBUG: Zone transition detected - from zone 1 to zone None
DEBUG: Robot position: world=(8.123, 4.567), pixel=(812, 456)
DEBUG: ===== ROBOT EXITED ALL COLLISION ZONES =====
DEBUG: Restoring default collision polygon points to: [[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]
DEBUG: Executing ROS2 command: ros2 param set /collision_monitor PolygonSlow.points [[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]
DEBUG: ✓ Successfully set collision polygon to: [[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]
```

**Expected**: White pixel detected, zone change to None, defaults restored

## Common Issues and Solutions

### Issue 1: No Position Updates

**Symptoms**:
```
DEBUG: Robot position not set yet (check #100)
```
(Repeats every ~20 seconds)

**Diagnosis**: Position updates not reaching collision monitor

**Check**:
1. Is robot connected? Look for "activate" lifecycle messages
2. Is `/diffbot_pose` topic publishing?
   ```bash
   ros2 topic echo /diffbot_pose
   ```
3. Add temporary debug in gui_manager.py emit_pose():
   ```python
   print(f"DEBUG: emit_pose called: ({x}, {y})")
   ```

### Issue 2: Pixel Colors Don't Match Zone Colors

**Symptoms**:
```
DEBUG: Pixel at (523, 345) has color RGB(200, 150, 150)
DEBUG: Pixel color RGB(200, 150, 150) doesn't match any zone
```

**Diagnosis**: Map image colors don't exactly match zone definition colors

**Possible Causes**:
1. PGM format converted colors (grayscale conversion)
2. Image compression artifacts
3. Wrong collision map being loaded

**Solution**:
1. Check the collision_<mapname>.pgm file visually - is it color or grayscale?
2. Verify collision_<mapname>_zones.json has correct color values
3. Try re-painting and saving zones

### Issue 3: Coordinates Out of Bounds

**Symptoms**:
```
DEBUG: Checking position - world=(5.234, 3.456), pixel=(-10, 500), map_size=(384, 384)
DEBUG: Robot position out of map bounds
```

**Diagnosis**: World-to-pixel coordinate conversion is wrong

**Check**:
1. Map origin in YAML file:
   ```bash
   cat ~/.robotroutes/maps/<mapname>.yaml
   ```
2. Robot position vs map origin - should robot be on this map?
3. Resolution value - typical is 0.05 (5cm per pixel)

**Common fix**: Robot is actually on a different map than loaded

### Issue 4: Collision Monitor Not Enabled

**Symptoms**:
```
DEBUG: CollisionMonitor not enabled (check #100)
```

**Diagnosis**: start_monitoring() never called

**Check**:
Look for this message after map load:
```
DEBUG: Collision monitor manager started for map: <mapname>
```

If missing, `_setup_collision_monitor()` may have failed silently.

### Issue 5: ROS2 Command Fails

**Symptoms**:
```
ERROR: ✗ Failed to set collision polygon!
ERROR: stderr: Node '/collision_monitor' not found
```

**Diagnosis**: collision_monitor node not running

**Solution**:
```bash
# Check if node is running
ros2 node list | grep collision_monitor

# Check node's parameters
ros2 param list /collision_monitor

# Verify parameter name exists
ros2 param get /collision_monitor PolygonSlow.points
```

## Quick Diagnostic Checklist

Run the robot and check off each item:

- [ ] Map loading shows collision files found
- [ ] Zone count > 0
- [ ] Zone colors listed (RGB values)
- [ ] "Started collision zone monitoring" message
- [ ] Position updates appear every 2-3 seconds
- [ ] "Checking position" messages appear every 10 seconds
- [ ] Pixel coordinates are within map bounds
- [ ] Pixel colors are being read (RGB values shown)
- [ ] When on painted area, pixel color matches zone color
- [ ] Zone transition message appears
- [ ] "ENTERED COLLISION ZONE" message appears
- [ ] ROS2 command execution shown
- [ ] ✓ Success message appears

If ALL are checked, zone detection is working!

## Filtering Debug Output

To see only collision zone messages:

```bash
ros2 launch roboto_viz gui_launch.py 2>&1 | grep -E "Collision|collision|zone|Zone|polygon"
```

To see position checking only:

```bash
ros2 launch roboto_viz gui_launch.py 2>&1 | grep "Checking position"
```

To see zone transitions only:

```bash
ros2 launch roboto_viz gui_launch.py 2>&1 | grep "ENTERED\|EXITED\|transition"
```
