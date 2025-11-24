#!/usr/bin/env python3
"""
Simple test script to verify battery SOC estimation logic.
Updated to use Modbus-based battery receiver.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from roboto_viz.modbus_battery_receiver import ModbusBatteryReceiver

def test_battery_conversion():
    """Test battery ADC to percentage conversion"""
    battery_receiver = ModbusBatteryReceiver()

    print("Testing battery SOC estimation (12-bit ADC, 0-4095):")
    print("=====================================================")

    # Test cases: (ADC value, expected voltage, expected percentage)
    # Note: 12-bit ADC (0-4095) instead of 10-bit (0-1023)
    test_cases = [
        (4095, 42.0, 100),  # Full battery (12-bit max)
        (0, 0.0, 0),        # Empty ADC
        (2048, 21.0, 0),    # Mid ADC but below min voltage
        (3121, 32.0, 0),    # Min safe voltage (32V / 42V * 4095)
        (3604, 37.0, 50),   # Mid range (37V / 42V * 4095)
        (400, 4.1, 0),      # Very low
        (408, 4.2, 0),      # Low but above test threshold for 10%
    ]

    # Calculate what ADC value gives us 10% (32V + 10% of 10V range = 33V)
    # 33V / 42V * 4095 = 3217 ADC
    adc_10_percent = int((33.0 / 42.0) * 4095)
    test_cases.append((adc_10_percent, 33.0, 10))  # 10% threshold
    
    for adc_value, expected_voltage, expected_percentage in test_cases:
        voltage = battery_receiver.adc_to_voltage(adc_value)
        percentage = battery_receiver.voltage_to_percentage(voltage)
        status_string = battery_receiver.get_battery_status_string(percentage, voltage)
        
        print(f"ADC: {adc_value:4d} -> Voltage: {voltage:4.1f}V -> Percentage: {percentage:3d}% -> Status: {status_string}")
        
        # Check if voltage conversion is approximately correct
        voltage_diff = abs(voltage - expected_voltage)
        if voltage_diff > 0.1:  # Allow 0.1V tolerance
            print(f"  WARNING: Voltage mismatch! Expected {expected_voltage}V, got {voltage}V")
    
    print("\nTesting warning threshold:")
    print("==========================")

    # Test warning cases (updated for 12-bit ADC)
    warning_test_cases = [
        (408, "Should show WARNING"),   # ~4.2V, very low
        (816, "Should show WARNING"),   # ~8.4V, low
        (1224, "Should show normal"),   # ~12.6V
        (2000, "Should show normal"),   # ~20.5V
        (4095, "Should show normal"),   # Max, 42V
    ]
    
    for adc_value, description in warning_test_cases:
        voltage = battery_receiver.adc_to_voltage(adc_value)
        percentage = battery_receiver.voltage_to_percentage(voltage)
        status_string = battery_receiver.get_battery_status_string(percentage, voltage)
        
        has_warning = "WARNING" in status_string
        print(f"ADC: {adc_value:4d} -> {percentage:3d}% -> {status_string:12s} -> {description} -> {'✓' if (has_warning and percentage <= 10) or (not has_warning and percentage > 10) else '✗'}")

if __name__ == "__main__":
    test_battery_conversion()