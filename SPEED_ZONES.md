# Speed Zone Editor Usage Guide

## Overview

The Speed Zone Editor allows you to create speed restriction zones on your robot's navigation map. These zones tell the robot to use specific speeds in certain areas, which is useful for:

- Narrow passages requiring slower movement
- High-traffic zones needing careful navigation
- Delicate environments requiring precision
- Areas where specific speed limits are needed

## How to Use

### 1. Access the Speed Zone Editor

1. Open the Plan Editor from the main GUI
2. Select and load a map using the "Load Map" button
3. Click the blue "Edit Speed Zones" button (located above the green "Save plan and exit" button)
4. A popup window will open with the Speed Zone Editor tools

### 2. Drawing Speed Zones

1. **Select Tool**: Choose "Paint Zone" to add speed restrictions or "Erase Zone" to remove them
2. **Set Speed Limit**: Choose from preset speed values (maximum 0.5 m/s):
   - 0.1 m/s (Very Slow) - For very tight spaces
   - 0.2 m/s (Slow) - For careful navigation
   - 0.3 m/s (Moderate) - Default balanced speed
   - 0.4 m/s (Fast) - For open areas needing some restriction
   - 0.5 m/s (Maximum) - Fastest allowed speed
3. **Draw Speed Zones**: Click and drag to draw rectangular speed zones on the map
4. **Visual Feedback**: Speed zones are only visible while editing

### 3. Speed Zone Colors

The system uses grayscale values to represent speed limits:
- **White (255)**: No speed restriction (robot uses default speed)
- **Very Dark Gray (~51)**: 0.1 m/s speed limit
- **Dark Gray (~102)**: 0.2 m/s speed limit  
- **Medium Gray (~153)**: 0.3 m/s speed limit
- **Light Gray (~204)**: 0.4 m/s speed limit
- **Very Light Gray (~255)**: 0.5 m/s speed limit

Higher gray values = higher allowed speeds

### 4. Map Operations

- **Load Original**: Restore the map to its original state (removes all speed zones)
- **Clear All Zones**: Remove all speed zones from the current map
- **Save Speed Mask**: Create `speed_mask.pgm` and `speed_mask.yaml` files for nav2
- **Close Speed Zone Editor**: Exit editing mode and hide speed zones from map display

### 5. Saving Your Work

When you click "Save Speed Mask", the system creates:
- `speed_mask.pgm`: The grayscale speed limit map
- `speed_mask.yaml`: Nav2-compatible metadata file with proper parameters

**Important**: Speed zones are only visible while the editor is open. When you close the editor, the map returns to normal display without showing the speed zones.

## Nav2 Integration

The generated files are compatible with nav2's speed restriction feature:

### speed_mask.yaml Parameters
```yaml
image: speed_mask.pgm
mode: scale                 # Use scale mode for grayscale interpretation
occupied_thresh: 1.0        # Full range conversion
free_thresh: 0.0           # Full range conversion  
resolution: [same as original map]
origin: [same as original map with zero yaw]
```

### Usage in Nav2

Add the speed restriction layer to your costmap configuration:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "speed_restriction_layer", "obstacle_layer", "inflation_layer"]
      speed_restriction_layer:
        plugin: "nav2_costmap_2d::SpeedFilterLayer"
        enabled: True
        filter_info_topic: "/speed_filter_info"
        
speed_filter_info:
  ros__parameters:
    speed_limit_topic: "/speed_limit"
    filter_info_topic: "/speed_filter_info"
    costmap_topic: "/local_costmap/costmap"
    base_value: 100.0          # Base speed value
    multiplier: -1.0           # Converts gray values to speed reductions
    filter_type: 0             # 0 = Speed Filter
    mask_yaml_file: "speed_mask.yaml"
```

## Tips

1. **Start with moderate speeds**: Begin with 0.3 m/s zones and adjust as needed
2. **Use rectangular zones**: Draw clean rectangular areas for consistent speed limits
3. **Test thoroughly**: Always test robot navigation after adding speed zones
4. **Keep backups**: The original map is automatically backed up as `[mapname]_original.pgm`
5. **Layer speeds logically**: Use faster speeds (0.4-0.5 m/s) for open areas, slower (0.1-0.2 m/s) for tight spaces

## Troubleshooting

- **Map not loading**: Ensure the map files (.pgm and .yaml) exist in `~/.robotroutes/maps/`
- **Drawing not working**: Make sure you clicked "Edit Speed Zones" first
- **Nav2 not recognizing zones**: Check that speed_mask.yaml has correct parameters
- **Robot ignoring zones**: Verify nav2 speed filter configuration is active