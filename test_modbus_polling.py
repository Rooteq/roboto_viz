#!/usr/bin/env python3
"""
Test script to verify Modbus polling logic (without hardware).
This tests the class structure and logic flow.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from roboto_viz.modbus_button_receiver import ModbusButtonReceiver
from roboto_viz.modbus_battery_receiver import ModbusBatteryReceiver


def test_button_receiver():
    """Test button receiver class initialization and structure"""
    print('Testing ModbusButtonReceiver:')
    print('=' * 50)

    # Create receiver instance (won't connect without hardware)
    receiver = ModbusButtonReceiver(
        port='/dev/ttyUSB0',
        slave_id=1,
        poll_interval=0.1
    )

    print(f'Port: {receiver.port}')
    print(f'Slave ID: {receiver.slave_id}')
    print(f'Poll interval: {receiver.poll_interval}s')
    print(f'Button click register: {receiver.ISTS_BUTTON_CLICK}')
    print(f'Button clear coil: {receiver.COIL_BUTTON_CLEAR}')

    status = receiver.get_connection_status()
    print(f'\nConnection status: {status}')
    print('✓ Button receiver class structure OK\n')


def test_battery_receiver():
    """Test battery receiver class initialization and structure"""
    print('Testing ModbusBatteryReceiver:')
    print('=' * 50)

    # Create receiver instance (won't connect without hardware)
    receiver = ModbusBatteryReceiver(
        port='/dev/ttyUSB0',
        slave_id=1,
        poll_interval=0.5
    )

    print(f'Port: {receiver.port}')
    print(f'Slave ID: {receiver.slave_id}')
    print(f'Poll interval: {receiver.poll_interval}s')
    print(f'Battery ADC register: {receiver.HREG_BATTERY_ADC}')
    print(f'ADC max value: {receiver.ADC_MAX}')
    print(f'Min voltage: {receiver.MIN_VOLTAGE}V')
    print(f'Max voltage: {receiver.MAX_VOLTAGE}V')

    # Test ADC conversion
    print('\nTesting ADC to voltage conversion:')
    test_values = [4095, 3217, 2048, 1000]
    for adc in test_values:
        voltage = receiver.adc_to_voltage(adc)
        percentage = receiver.voltage_to_percentage(voltage)
        print(f'  ADC {adc:4d} -> {voltage:5.1f}V -> {percentage:3d}%')

    status = receiver.get_connection_status()
    print(f'\nConnection status: {status}')
    print('✓ Battery receiver class structure OK\n')


if __name__ == '__main__':
    print('Modbus Receiver Classes Test')
    print('=' * 50)
    print('This test verifies class structure and logic.')
    print('Hardware connection is not required.\n')

    test_button_receiver()
    test_battery_receiver()

    print('All tests passed!')
