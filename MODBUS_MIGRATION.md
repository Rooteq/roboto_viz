# Migration from CAN to Modbus for Button and Battery Polling

## Overview

This document describes the migration from CAN bus to Modbus RTU for button click detection and battery voltage monitoring in the roboto_viz ROS2 package.

## Changes Made

### New Files Created

1. **[roboto_viz/modbus_button_receiver.py](roboto_viz/modbus_button_receiver.py)**
   - Replaces `can_signal_receiver.py`
   - Polls discrete input register for button click flag (address 0)
   - Clears button flag via coil write (address 6) after detection
   - Uses threading for background polling
   - Configurable poll interval (default: 0.1s)

2. **[roboto_viz/modbus_battery_receiver.py](roboto_viz/modbus_battery_receiver.py)**
   - Replaces `can_battery_receiver.py`
   - Polls holding register for 12-bit battery ADC value (address 0)
   - Converts ADC (0-4095) to voltage and percentage
   - Uses median filtering on 15 samples
   - Pauses updates during navigation
   - Configurable poll interval (default: 0.5s)

### Modified Files

1. **[roboto_viz/gui_manager.py](roboto_viz/gui_manager.py)**
   - Updated imports to use new Modbus receivers
   - Changed `can_battery_receiver` → `modbus_battery_receiver`
   - Changed `can_signal_receiver` → `modbus_button_receiver`
   - Renamed `_connect_can_signals()` → `_connect_modbus_signals()`
   - Updated `trigger_configure()` to start Modbus receivers
   - Updated `trigger_deactivate()` to stop Modbus receivers
   - Added poll interval parameters to `__init__`

2. **[test_battery_conversion.py](test_battery_conversion.py)**
   - Updated to use `ModbusBatteryReceiver`
   - Updated test cases for 12-bit ADC (0-4095) instead of 10-bit (0-1023)

### Test Files

1. **[test_modbus_polling.py](test_modbus_polling.py)** (new)
   - Tests Modbus receiver class structure and logic
   - Verifies ADC conversion without hardware

## Modbus Register Mapping

### Button Receiver
- **Discrete Input 0**: Button click flag (read with function code 2)
- **Coil 6**: Button clear flag (write with function code 5)

### Battery Receiver
- **Holding Register 0**: Battery ADC value (read with function code 3)
- **ADC Range**: 0-4095 (12-bit ESP32 ADC)
- **Voltage Range**: 32.0V (0%) to 42.0V (100%)

## Configuration

Both receivers use the same Modbus interface as the existing LED/buzzer manager:

```python
GuiManager(
    modbus_port='/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if00-port0',
    modbus_slave_id=1,
    button_poll_interval=0.1,    # 100ms button polling
    battery_poll_interval=0.5    # 500ms battery polling
)
```

## Polling Logic

### Button Polling
```
Loop every 0.1s:
  1. Read discrete input 0 (button click flag)
  2. If flag is set:
     - Increment click counter
     - Emit PyQt signal (signal_received)
     - Write coil 6 to clear flag
```

### Battery Polling
```
Loop every 0.5s:
  1. Read holding register 0 (battery ADC)
  2. Add to median filter buffer (15 samples)
  3. If not navigating and 5s elapsed since last update:
     - Calculate median ADC
     - Convert to voltage: V = (ADC / 4095) * 3.6 * (42/3.6)
     - Convert to percentage: linear scale 32V-42V
     - Apply 90% → 100% scaling
     - Emit PyQt signals if percentage changed
```

## Advantages Over CAN

1. **Single Interface**: Uses same Modbus connection as LED/buzzer control
2. **Simpler Wiring**: No separate CAN bus required
3. **Polling Control**: Easy to adjust polling rates per signal
4. **Standard Protocol**: Modbus RTU is widely supported
5. **Clear API**: Read/write operations are explicit

## Backward Compatibility

- Old CAN receiver classes remain in codebase for reference
- Signal names and behavior unchanged (same PyQt signals)
- GUI Manager parameter `enable_can` still controls Modbus initialization
- All downstream code continues to work without changes

## Testing

To verify the implementation without hardware:

```bash
# Test battery conversion logic
python3 test_battery_conversion.py

# Test Modbus class structure
python3 test_modbus_polling.py

# Build the package
colcon build --packages-select roboto_viz

# Check code style
ament_flake8 roboto_viz/
```

## ESP32 Firmware Requirements

The ESP32 firmware must implement the following Modbus registers:

1. **Discrete Input 0** (ISTS_BUTTON_CLICK)
   - Read-only
   - Set to 1 when button is clicked
   - Application must poll and clear via coil 6

2. **Coil 6** (COIL_BUTTON_CLEAR)
   - Write-only
   - Writing 1 clears the button click flag

3. **Holding Register 0** (HREG_BATTERY_ADC)
   - Read-only
   - Contains 12-bit ADC reading (0-4095)
   - Updated by ESP32 ADC reading task

## Reference Script

The original polling script that inspired this implementation:

```python
# From user-provided button_poll.py
import minimalmodbus

ISTS_BUTTON_CLICK = 0
COIL_BUTTON_CLEAR = 6
HREG_BATTERY_ADC = 0

instr = minimalmodbus.Instrument(port, slave_id)
instr.serial.baudrate = 9600
instr.mode = minimalmodbus.MODE_RTU

# Button polling
if instr.read_bit(ISTS_BUTTON_CLICK, functioncode=2):
    print('Button clicked!')
    instr.write_bit(COIL_BUTTON_CLEAR, 1, functioncode=5)

# Battery polling
adc_value = instr.read_register(HREG_BATTERY_ADC, functioncode=3)
voltage = (adc_value / 4095.0) * 3.6
```

## Migration Checklist

- [x] Create ModbusButtonReceiver class
- [x] Create ModbusBatteryReceiver class
- [x] Update gui_manager.py imports
- [x] Update gui_manager.py initialization
- [x] Update gui_manager.py signal connections
- [x] Update test_battery_conversion.py
- [x] Create test_modbus_polling.py
- [x] Fix flake8 style issues
- [x] Build and verify package
- [x] Test battery conversion logic
- [x] Document changes

## Next Steps

1. Test with actual ESP32 hardware
2. Verify button click detection works
3. Verify battery ADC reading matches expected values
4. Tune poll intervals if needed
5. Monitor for Modbus timeout errors
6. Consider removing old CAN receiver files after validation
