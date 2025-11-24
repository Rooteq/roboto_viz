# Battery Voltage Range Update

## Change Summary

Updated battery voltage range to match actual ADC input measurements:
- **Previous**: 32V - 42V (assumed battery pack voltage with divider)
- **New**: 2.182V - 3.204V (direct ADC input voltage)

## Rationale

The ESP32 ADC is measuring a scaled-down voltage directly (likely from a voltage divider), not the full battery pack voltage. The voltage range 2.182V to 3.204V represents the actual ADC input signal corresponding to 0% to 100% battery charge.

## Changes Made

### [modbus_battery_receiver.py](roboto_viz/modbus_battery_receiver.py)

1. **Updated voltage constants:**
   ```python
   # Old:
   self.MAX_VOLTAGE = 42.0   # 100% - 10S Li-ion full charge
   self.MIN_VOLTAGE = 32.0   # 0% - lowest safe voltage

   # New:
   self.MAX_VOLTAGE = 3.204  # 100% battery - ADC input voltage
   self.MIN_VOLTAGE = 2.182  # 0% battery - ADC input voltage
   ```

2. **Simplified `adc_to_voltage()` method:**
   - Removed battery voltage scaling logic
   - Now returns direct ADC input voltage (0-3.6V range)
   - No more voltage divider ratio calculation

3. **Simplified `voltage_to_percentage()` method:**
   - Removed 90% → 100% scaling hack
   - Now uses simple linear interpolation: `(V - 2.182) / (3.204 - 2.182) * 100`
   - Clean 0-100% mapping

4. **Updated `MIN_ADC_THRESHOLD`:**
   - Calculated from actual min voltage: `(2.182V / 3.6V) * 4095 = ~2481 ADC`
   - Minus 100 ADC buffer = 2381 ADC threshold
   - Rejects readings below ~2.1V

### [test_battery_conversion.py](test_battery_conversion.py)

Complete rewrite to test new voltage range:
- Tests key percentages: 0%, 10%, 50%, 100%
- Validates voltage-to-percentage conversion
- Shows full voltage mapping table
- All tests pass ✓

## ADC to Percentage Mapping

| ADC Value | Voltage | Battery % |
|-----------|---------|-----------|
| < 2382    | < 2.182V | 0% (rejected) |
| 2482      | 2.182V  | 0%        |
| 2598      | 2.284V  | 10%       |
| 3063      | 2.693V  | 50%       |
| 3644      | 3.204V  | 100%      |
| > 3644    | > 3.204V | 100% (clamped) |

## Voltage Range Details

- **ADC Resolution**: 12-bit (0-4095)
- **ADC Voltage Range**: 0 - 3.6V (ESP32 ADC_11db attenuation)
- **Battery Voltage Range**: 2.182V - 3.204V (1.022V span)
- **Percentage Resolution**: ~0.25% per ADC count

## Testing

```bash
# Test conversion logic
python3 test_battery_conversion.py

# Expected output:
# All test cases pass with ✓
# Voltage mapping matches expected values
# MIN_ADC_THRESHOLD = 2382 ADC
```

## Behavior Changes

### Before
- Battery showed unrealistic voltages (32V-42V)
- Used 90% scaling hack (90% voltage = 100% display)
- Complex voltage divider calculations

### After
- Battery shows actual ADC input voltage (2.182V-3.204V)
- Linear 0-100% mapping, no scaling hacks
- Simple, direct voltage conversion

## Calibration

If the battery percentage readings don't match real battery state:

1. **Measure actual voltages** at 0% and 100% battery with multimeter at ADC input
2. **Update constants** in `modbus_battery_receiver.py`:
   ```python
   self.MIN_VOLTAGE = <measured_0%_voltage>
   self.MAX_VOLTAGE = <measured_100%_voltage>
   ```
3. **Rebuild and test:**
   ```bash
   colcon build --packages-select roboto_viz
   python3 test_battery_conversion.py
   ```

## Notes

- The voltage divider ratio from battery pack to ADC input is ~13.1x (42V pack → 3.204V)
- This is transparent to the software - we only care about ADC input voltage
- ESP32 firmware must output raw 12-bit ADC value (0-4095) via Modbus
- No firmware changes required - this is a software-only update

## Compatibility

- **Forward compatible**: Old data will show as 100% (voltage > 3.204V)
- **Backward compatible**: Code still accepts full 0-4095 ADC range
- **No breaking changes**: PyQt signals unchanged, same data types
