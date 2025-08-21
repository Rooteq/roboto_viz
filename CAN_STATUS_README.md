# CAN RGB LED Control System

This system provides automatic CAN bus messaging to control RGB status LEDs on the robot based on all status signals in the roboto_viz application.

## Overview

The CAN LED control system monitors all robot status signals and sends simple CAN messages to control RGB LEDs. When any status changes, it automatically determines the appropriate LED color (Green/Orange/Red) and sends an empty CAN frame with the corresponding ID.

## Features

- **Automatic LED Control**: Monitors all existing status signals and controls LEDs
- **Simple Architecture**: Just 3 CAN IDs for 3 LED colors
- **Status Level Classification**: Automatically categorizes status as OK, WARNING, or ERROR
- **Change-Based Messaging**: Only sends LED commands when status level changes
- **All Status Types**: Supports robot, navigation, docking, battery, plan, and manual status

## CAN Message Format

Each CAN message is an **empty frame (0 bytes)** - only the CAN ID is used to trigger the LED:

```
CAN ID: LED color identifier
Data Length: 0 bytes (empty frame)
Data: None
```

## CAN Message IDs for LED Control

| LED Color | CAN ID | Status Level | When Sent |
|-----------|--------|--------------|-----------|
| Green LED | 0x201 | OK | Robot operating normally |
| Orange LED | 0x202 | WARNING | Warning conditions |
| Red LED | 0x203 | ERROR | Error/failure conditions |

## Status Level Mapping

The system automatically determines status levels based on status text:

### OK (Level 0)
- "Idle", "At base", "At destination"
- "Docked", "Undocked", "Connected", "Available"
- Navigation states: "Nav to base", "Nav to dest", "Navigating"
- Execution states: "Executing", "Manual move"

### WARNING (Level 1)
- "Warning", "Low battery", "Obstacle detected"
- "Waiting for signal"

### ERROR (Level 2)
- "Failed", "Error", "Connection lost"
- "Navigation Error", "Navigation failed"
- "Dock failed", "Undock failed", "Not available"

## Usage

### 1. Enable CAN in GUI Manager

The CAN system is automatically integrated when you create a GuiManager instance:

```python
# Enable CAN with default interface (can0)
gui_manager = GuiManager(enable_can=True)

# Use custom CAN interface
gui_manager = GuiManager(can_interface="vcan0", enable_can=True)

# Disable CAN
gui_manager = GuiManager(enable_can=False)
```

### 2. CAN Interface Setup

Create a CAN interface (example with virtual CAN):

```bash
# Create virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Monitor messages
candump vcan0
```

### 3. Adding New Status Types

To add a new status type:

1. **Add CAN ID** in `can_status_manager.py`:
```python
class CANStatusType(IntEnum):
    # ... existing types ...
    NEW_STATUS = 0x207
```

2. **Add Handler** in `CANStatusManager`:
```python
@pyqtSlot(str)
def handle_new_status(self, status: str):
    """Handle new status updates"""
    self.send_status_if_changed(CANStatusType.NEW_STATUS, status)
```

3. **Add Signal** in `GuiManager`:
```python
newStatusCAN = pyqtSignal(str)
```

4. **Connect Signal** in `_connect_can_signals()`:
```python
self.newStatusCAN.connect(self.can_manager.handle_new_status)
```

5. **Add Forwarder** in integration helper:
```python
forwarders['new_status'] = lambda status: gui_manager.newStatusCAN.emit(status)
```

### 4. Adding New Status Mappings

Add custom status level mappings dynamically:

```python
# In your code
gui_manager.can_manager.add_status_mapping("custom error", StatusLevel.ERROR)
gui_manager.can_manager.add_status_mapping("custom warning", StatusLevel.WARNING)
```

## Testing

### Use the Test Application

```bash
# Run the CAN LED tester
python3 roboto_viz/test_can_status.py
```

The tester provides a GUI to:
- Connect/disconnect from CAN interface
- Send direct LED control messages (Green/Orange/Red)
- Test status-to-LED mapping with various robot status examples

### Monitor CAN Messages

```bash
# Monitor all CAN LED messages
candump can0

# Monitor specific LED (e.g., green LED)
candump can0,201:7FF

# Monitor all LED IDs
candump can0,201:203

# Log LED messages to file
candump can0 -L
```

### Decode CAN LED Messages

Example CAN LED message decoding:

```bash
# CAN message: can0  201   [0]
# ID: 0x201 (Green LED)
# Data Length: 0 (empty frame)
# Meaning: Turn on Green LED (robot status OK)

# CAN message: can0  203   [0]  
# ID: 0x203 (Red LED)
# Data Length: 0 (empty frame)
# Meaning: Turn on Red LED (robot error status)
```

## Architecture

### Core Components

1. **CANStatusManager**: Manages CAN connection and message transmission
2. **CANStatusType**: Defines CAN message IDs for different status types
3. **StatusLevel**: Defines status severity levels (OK/WARNING/ERROR)
4. **can_integration_helper**: Provides easy integration with existing GUI

### Integration Points

- **GuiManager**: Contains CAN manager instance and forwarding signals
- **State Machine**: Connects status signals to CAN forwarding in ActiveState and PlanActiveState
- **Views**: Status updates automatically trigger CAN messages via signal connections

## Configuration

### Environment Variables

You can configure the CAN interface via environment variable:

```bash
export ROBOTO_CAN_INTERFACE=vcan0
```

### Runtime Configuration

```python
# Check CAN status
info = gui_manager.can_manager.get_status_info()
print(f"CAN connected: {info['connected']}")
print(f"Interface: {info['interface']}")

# Add custom status mapping
gui_manager.can_manager.add_status_mapping("my_custom_status", StatusLevel.WARNING)
```

## Troubleshooting

### Common Issues

1. **CAN interface not found**
   - Ensure CAN interface exists: `ip link show`
   - Check interface is up: `ip link set up can0`

2. **Permission denied**
   - Add user to dialout group: `sudo usermod -a -G dialout $USER`
   - Or run with sudo (not recommended for production)

3. **No CAN messages appearing**
   - Check CAN connections are established in state machine
   - Verify GUI manager has `enable_can=True`
   - Check console for connection errors

### Debug Information

Enable debug output:

```python
# Print CAN manager status
info = gui_manager.can_manager.get_status_info()
print(info)

# Monitor connections in state machine
print("CAN forwarding connections established")
```

## Example CAN LED Message Flows

### Normal Operation Flow
```
1. Robot starts navigation → "Nav to dest"
2. GuiManager.navigator.navStatus.emit("Nav to dest")  
3. Status classified as OK (Level=0)
4. CAN LED message sent: ID=0x201 (Green LED) - empty frame
5. Robot reaches destination → "At destination"  
6. Status still OK - no additional CAN message (no change)
```

### Error Status Flow
```
1. Navigation fails → "Navigation failed"
2. Status classified as ERROR (Level=2)
3. CAN LED message sent: ID=0x203 (Red LED) - empty frame
4. Robot recovers → "Idle"
5. Status changes to OK (Level=0)
6. CAN LED message sent: ID=0x201 (Green LED) - empty frame
```

### Warning Status Flow
```
1. Battery gets low → "Low battery"
2. Status classified as WARNING (Level=1)
3. CAN LED message sent: ID=0x202 (Orange LED) - empty frame
4. Battery charged → "Battery normal"
5. Status changes to OK (Level=0)  
6. CAN LED message sent: ID=0x201 (Green LED) - empty frame
```

This system provides a simple, automatic way to control RGB status LEDs via CAN bus based on all robot status information, giving immediate visual feedback of robot state.