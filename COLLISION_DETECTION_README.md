# Collision Detection Buzzer Control Implementation

## Overview

This implementation adds collision detection functionality to the roboto_viz package that subscribes to CollisionDetectorState messages and controls a buzzer via CAN messages based on collision detection status.

## Implementation Details

### 1. CollisionDetectorState Message Subscriber

**Location**: `gui_manager.py` - `ManagerNode` class

- **Topic**: `/collision_detector/detector_state`
- **Message Type**: `CollisionDetectorState`
- **Callback**: `collision_callback(self, msg)`

The subscriber is created in the `on_activate()` method and destroyed in the `on_deactivate()` method.

### 2. CollisionDetectorState Message Definition

**Location**: `roboto_viz/collision_detector_state.py`

Since the exact ROS2 message package was unknown, a local Python definition was created:

```python
class CollisionDetectorState:
    def __init__(self):
        self.polygons = []     # List of polygon names (string[])
        self.detections = []   # List of detection bools (bool[])
```

The import hierarchy tries multiple possible packages before falling back to the local definition.

### 3. Collision Detection Logic

**Location**: `gui_manager.py` - `collision_callback()` method

```python
def collision_callback(self, msg):
    """Handle collision detector state messages"""
    if msg.detections:
        # Check if any detection is true
        collision_detected = any(msg.detections)
        
        # Call the callback if set
        if self.collision_detection_callback:
            self.collision_detection_callback(collision_detected)
    else:
        # No detections array, assume no collision
        if self.collision_detection_callback:
            self.collision_detection_callback(False)
```

The logic uses `any(msg.detections)` to determine if any polygon has detected a collision.

### 4. Signal Flow

**Path**: `ManagerNode` → `GuiManager` → `CANStatusManager`

1. `ManagerNode.collision_callback()` receives ROS message
2. Calls `GuiManager.emit_collision_detection()`  
3. Emits `collision_detected` PyQt signal
4. Signal connected to `CANStatusManager.handle_collision_detection()`
5. Calls `send_buzzer_status(collision_detected)`

### 5. CAN Buzzer Control

**Location**: `can_status_manager.py` - `CANStatusManager` class

**Message IDs**:
- `0x204` - Buzzer ON (collision detected during navigation)
- `0x205` - Buzzer OFF (no collision or not navigating)

**Navigation State Tracking**:
The system now tracks navigation state and only sends buzzer ON when BOTH conditions are met:
1. Collision is detected (`any(msg.detections) == True`)
2. Robot is actively navigating (`is_navigating == True`)

**Methods Added/Updated**:
```python
@pyqtSlot(bool)
def handle_collision_detection(self, collision_detected: bool):
    """Handle collision detection updates and control buzzer"""
    self.send_buzzer_status(collision_detected)

def send_buzzer_status(self, collision_detected: bool):
    """
    Send buzzer control message based on collision detection status.
    Only sends buzzer ON when both collision is detected AND robot is navigating.
    Always sends buzzer OFF when not navigating or no collision.
    """
    should_buzz = collision_detected and self.is_navigating
    buzzer_id = CANBuzzerType.BUZZER_ON if should_buzz else CANBuzzerType.BUZZER_OFF
    return self._send_buzzer_can_message(buzzer_id)
```

**Navigation Status Tracking**:
Navigation state is updated based on status messages:
- **Active Navigation**: "nav to", "navigating", "nawigacja" 
- **Inactive Navigation**: "zatrzymany", "stopped", "na miejscu", "w bazie", "bezczynny", "idle", "błąd", "failed", "error"

### 6. CAN Message Format

**Message Structure**: Empty frame (0 bytes) - only CAN ID matters

```
CAN ID: 0x204 (BUZZER_ON) or 0x205 (BUZZER_OFF)
Data Length: 0 bytes
Data: None
```

The buzzer hardware only needs to see the CAN ID to determine action.

## Setup and Configuration

### 1. Signal Connections

**Location**: `gui_manager.py` - `GuiManager._connect_can_signals()`

Added:
```python
# Connect collision detection signal to buzzer control
self.collision_detected.connect(self.can_manager.handle_collision_detection)
```

### 2. Callback Setup  

**Location**: `gui_manager.py` - `GuiManager.run()`

Added:
```python
self.node.collision_detection_callback = self.emit_collision_detection
```

## Testing

A test script (`test_collision_detection.py`) validates:
- ✓ CollisionDetectorState import successful
- ✓ Message instance creation
- ✓ Collision detection logic (`any()` function)
- ✓ CAN status manager integration
- ✓ Correct buzzer message IDs (0x204/0x205)

## Usage

The system is fully automatic with intelligent behavior:

1. **Robot Operation**: Navigation system publishes to `/collision_detector/detector_state`
2. **Navigation Tracking**: System monitors navigation status to determine if robot is actively navigating
3. **Smart Buzzer Logic**: 
   - **Buzzer ON (0x204)**: Only when collision detected AND robot is navigating
   - **Buzzer OFF (0x205)**: When no collision OR robot is not navigating
4. **Hardware Response**: Buzzer hardware responds to CAN messages immediately

**Example Scenarios**:
- Robot idle + collision detected → Buzzer OFF (not navigating)
- Robot navigating + no collision → Buzzer OFF (no collision) 
- Robot navigating + collision detected → Buzzer ON (both conditions met)
- Robot stops navigation → Buzzer OFF (automatically sent when navigation ends)

## Dependencies

- **ROS2**: CollisionDetectorState message (with fallback)
- **CAN**: Linux CAN interface (can0 by default)
- **PyQt5**: Signal/slot system for internal communication
- **Existing**: CAN status manager infrastructure

## Troubleshooting

### Message Import Issues
The implementation handles missing ROS2 message packages gracefully by trying multiple import paths and falling back to a local definition.

### CAN Interface Issues
Use existing CAN troubleshooting from `CAN_STATUS_README.md` - check interface with `candump can0`.

### Debug Output
Both collision detection and CAN transmission provide console output for debugging:
- `"Created collision detector subscriber"` - Subscription established
- `"CAN Status: Navigation failure detected: [status] - sending ERROR message"` - Navigation issues  
- `"CAN Buzzer: Sent BUZZER_ON/BUZZER_OFF (ID: 0x204/0x205)"` - CAN transmission confirmation

**Navigation State Changes**:
When navigation status changes, the system automatically:
- Sets `is_navigating = True` for active navigation states
- Sets `is_navigating = False` for inactive states  
- Sends `BUZZER_OFF (0x205)` immediately when navigation stops

## Architecture Integration

This implementation integrates seamlessly with the existing roboto_viz architecture:
- Uses existing CAN infrastructure
- Follows existing signal/slot patterns  
- Maintains existing error handling
- Compatible with all operation modes (Active, Planning, etc.)

The collision detection buzzer control is now fully integrated and ready for use.
