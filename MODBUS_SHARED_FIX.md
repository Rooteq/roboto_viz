# Modbus Serial Port Conflict Fix

## Problem Diagnosed

The original implementation had **3 separate Modbus connections** all trying to access the **same serial port simultaneously**:

1. `CANStatusManager` (LED/buzzer) - continuous 1Hz messages
2. `ModbusBatteryReceiver` - polling every 0.5s
3. `ModbusButtonReceiver` - polling every 0.1s

This caused massive collisions:
- Checksum errors (responses from one request picked up by another)
- "Device disconnected" errors (port already in use)
- Corrupted Modbus frames
- Communication failures

## Solution: Shared Instrument with Locking

All three components now share a **single Modbus instrument** with a **threading.Lock** for mutual exclusion:

```python
# One instrument created by CANStatusManager
instrument = minimalmodbus.Instrument(port, slave_id)
instrument_lock = threading.Lock()

# Shared with battery and button receivers
battery_receiver = ModbusBatteryReceiver(
    shared_instrument=instrument,
    instrument_lock=instrument_lock
)
button_receiver = ModbusButtonReceiver(
    shared_instrument=instrument,
    instrument_lock=instrument_lock
)
```

Every Modbus operation now uses `with self.instrument_lock:` to ensure serial access.

## Changes Made

### 1. [env_expression_manager.py](roboto_viz/env_expression_manager.py)
- Added `threading.Lock()` as `self.instrument_lock`
- Wrapped all `instrument.read_bit()` / `instrument.write_bit()` calls with lock
- Exposed `instrument` and `instrument_lock` for sharing

### 2. [modbus_button_receiver.py](roboto_viz/modbus_button_receiver.py)
- Removed independent Modbus connection logic
- Changed `__init__` to accept `shared_instrument` and `instrument_lock`
- All Modbus operations now use `with self.instrument_lock:`
- Increased default poll interval from 0.1s → 1.0s (less aggressive)

### 3. [modbus_battery_receiver.py](roboto_viz/modbus_battery_receiver.py)
- Removed independent Modbus connection logic
- Changed `__init__` to accept `shared_instrument` and `instrument_lock`
- All Modbus operations now use `with self.instrument_lock:`
- Increased default poll interval from 0.5s → 2.0s (less aggressive)
- Suppressed "ADC below threshold" spam

### 4. [gui_manager.py](roboto_viz/gui_manager.py)
- Create `CANStatusManager` first (owns the instrument and lock)
- Pass `can_manager.instrument` and `can_manager.instrument_lock` to receivers
- In `trigger_configure()`, share instrument after connection:
  ```python
  can_manager.connect_can()
  battery_receiver.instrument = can_manager.instrument
  button_receiver.instrument = can_manager.instrument
  ```

## New Poll Intervals

Reduced polling rates to minimize bus contention:

| Component | Old Interval | New Interval | Reason |
|-----------|--------------|--------------|--------|
| LED Manager | 1Hz continuous | 1Hz continuous | Unchanged |
| Battery ADC | 0.5s (2Hz) | 2.0s (0.5Hz) | Battery changes slowly |
| Button Click | 0.1s (10Hz) | 1.0s (1Hz) | Human reaction time sufficient |

This gives ~3-4 Modbus transactions per second instead of ~13, reducing collisions dramatically.

## Thread Safety

All Modbus access is now properly serialized:

```python
with self.instrument_lock:
    result = self.instrument.read_bit(address, functioncode=2)
```

This ensures only one thread can access the serial port at a time, preventing:
- Interleaved bytes from concurrent writes
- One thread reading another's response
- Checksum corruption from overlapping frames

## Testing

Build successful:
```bash
cd /home/amr/ros2_ws
colcon build --packages-select roboto_viz
```

Expected behavior:
- Clean Modbus communication with no checksum errors
- All three functions (LED, battery, button) work correctly
- No "device disconnected" warnings
- Battery and button updates at slower but acceptable rates

## Performance Impact

- **Battery**: Updates every 2s instead of 0.5s (still plenty fast for Li-ion)
- **Button**: Polls every 1s instead of 0.1s (might miss rapid double-clicks, but acceptable for "wait" actions)
- **LEDs**: Unchanged at 1Hz continuous

If button responsiveness is insufficient, increase rate to 0.5s (2Hz) after verifying no collisions.

## Configuration

Poll intervals can be adjusted in `gui_app/app.py` or wherever `GuiManager` is instantiated:

```python
gui_manager = GuiManager(
    button_poll_interval=1.0,    # 1Hz button polling
    battery_poll_interval=2.0    # 0.5Hz battery polling
)
```

Recommended ranges:
- **Button**: 0.5s - 2.0s (faster = more responsive, more collisions)
- **Battery**: 1.0s - 5.0s (Li-ion changes slowly, no need for sub-second)

## Troubleshooting

If you still see occasional errors:
1. **Increase poll intervals** - Give more time between operations
2. **Increase Modbus timeout** - Currently 0.5s, could go to 1.0s
3. **Check baud rate** - Should be 9600, matching ESP32
4. **Check cable** - Poor connections cause intermittent errors

## Key Takeaway

**Never open multiple connections to the same serial port.** Always share a single connection with proper locking.
