#!/usr/bin/env python3
"""
Simple test script to verify battery SOC estimation logic.
Updated for new voltage range: 2.182V = 0%, 3.204V = 100%
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from roboto_viz.modbus_battery_receiver import ModbusBatteryReceiver


def test_battery_conversion():
    """Test battery ADC to percentage conversion"""
    # Create receiver with dummy parameters (not used for conversion test)
    battery_receiver = ModbusBatteryReceiver(
        shared_instrument=None,
        instrument_lock=None,
        poll_interval=1.0
    )

    print('Testing battery SOC estimation (12-bit ADC, 0-4095):')
    print('Voltage range: 2.182V (0%) to 3.204V (100%)')
    print('=' * 60)

    # Calculate ADC values for key voltages
    # ADC = (voltage / 3.6V) * 4095
    adc_0_percent = int((2.182 / 3.6) * 4095)    # ~2481 ADC
    adc_10_percent = int((2.284 / 3.6) * 4095)   # ~2598 ADC (10% voltage)
    adc_50_percent = int((2.693 / 3.6) * 4095)   # ~3064 ADC (50% voltage)
    adc_100_percent = int((3.204 / 3.6) * 4095)  # ~3645 ADC

    # Test cases: (ADC value, expected voltage, expected percentage)
    test_cases = [
        (0, 0.0, 0),                    # Empty ADC
        (adc_0_percent, 2.182, 0),      # 0% battery
        (adc_10_percent, 2.284, 10),    # 10% battery
        (adc_50_percent, 2.693, 50),    # 50% battery
        (adc_100_percent, 3.204, 100),  # 100% battery
        (4095, 3.6, 100),               # Max ADC (clamped to 100%)
        (2000, 1.76, 0),                # Below min voltage
    ]

    print('\nTest cases:')
    print('-' * 60)
    for adc_value, expected_voltage, expected_percentage in test_cases:
        voltage = battery_receiver.adc_to_voltage(adc_value)
        percentage = battery_receiver.voltage_to_percentage(voltage)
        status_string = battery_receiver.get_battery_status_string(percentage)

        status = '✓' if abs(percentage - expected_percentage) <= 1 else '✗'
        print(f'{status} ADC: {adc_value:4d} -> '
              f'{voltage:5.3f}V -> {percentage:3d}% -> {status_string}')

        # Check if voltage conversion is approximately correct
        voltage_diff = abs(voltage - expected_voltage)
        if voltage_diff > 0.02:  # Allow 20mV tolerance
            print(f'   WARNING: Voltage mismatch! '
                  f'Expected {expected_voltage:.3f}V, got {voltage:.3f}V')

    print('\nVoltage to percentage mapping:')
    print('-' * 60)
    test_voltages = [2.0, 2.182, 2.5, 2.693, 3.0, 3.204, 3.5]
    for voltage in test_voltages:
        percentage = battery_receiver.voltage_to_percentage(voltage)
        adc = int((voltage / 3.6) * 4095)
        print(f'{voltage:.3f}V -> {percentage:3d}% (ADC: {adc:4d})')

    print('\nVoltage range:')
    print('-' * 60)
    print(f'Min voltage: {battery_receiver.MIN_VOLTAGE}V (0%)')
    print(f'Max voltage: {battery_receiver.MAX_VOLTAGE}V (100%)')
    print(f'Voltage span: {battery_receiver.MAX_VOLTAGE - battery_receiver.MIN_VOLTAGE:.3f}V')
    print(f'Simplified design - no thresholds, breaks, or filtering delays')


if __name__ == '__main__':
    test_battery_conversion()
