#!/usr/bin/env python3
"""Modbus-based battery ADC receiver for ESP32 AMR Sound Viz Controller.

Polls for battery voltage using Modbus RTU.
Replaces CAN-based battery receiver (can_battery_receiver.py)
Uses shared Modbus instrument to avoid serial port conflicts.
"""

from collections import deque
import statistics
import threading
import time

import minimalmodbus
from PyQt5.QtCore import pyqtSignal, QObject


class ModbusBatteryReceiver(QObject):
    """Receives battery ADC readings from ESP32 via Modbus RTU.

    Polls holding register for battery ADC value (12-bit, 0-4095).
    Converts to voltage and percentage for display.
    Updates are paused during navigation.
    """

    battery_status_update = pyqtSignal(int)  # Raw ADC (0-4095)
    battery_percentage_update = pyqtSignal(int, str)  # % (0-100), str

    # Modbus register addresses (must match relay_modbus.h)
    HREG_BATTERY_ADC = 0       # Holding register for battery ADC value

    def __init__(self, shared_instrument, instrument_lock, poll_interval=2.0):
        """Initialize battery receiver with shared Modbus instrument.

        Args:
            shared_instrument: Shared minimalmodbus.Instrument instance
            instrument_lock: threading.Lock for serializing access
            poll_interval: Polling interval in seconds (default: 2.0s)
        """
        super().__init__()
        self.instrument = shared_instrument
        self.instrument_lock = instrument_lock
        self.poll_interval = poll_interval
        self.receiving = False
        self.receive_thread = None

        # Simple median filter
        self.sample_buffer = deque(maxlen=10)  # Keep 10 recent samples
        self.current_percentage = None

        # Navigation state - updates paused when navigating
        self.is_navigating = False

        # Battery voltage constants - ADC input voltage range
        # ESP32 ADC is 12-bit (0-4095) with 11db attenuation (~3.6V max)
        self.ADC_MAX = 4095.0
        self.ADC_VOLTAGE_MAX = 3.6
        self.MIN_VOLTAGE = 2.182  # 0% battery
        self.MAX_VOLTAGE = 3.204  # 100% battery

    def set_navigation_state(self, is_navigating: bool):
        """Update navigation state - battery updates pause during navigation.

        Args:
            is_navigating: True if robot is currently navigating
        """
        self.is_navigating = is_navigating

    def adc_to_voltage(self, adc_value: int) -> float:
        """Convert 12-bit ADC value to input voltage (V)."""
        return (adc_value / self.ADC_MAX) * self.ADC_VOLTAGE_MAX

    def voltage_to_percentage(self, voltage: float) -> int:
        """Convert ADC input voltage to battery percentage (0-100).

        2.182V = 0%, 3.204V = 100%, linear scaling.
        """
        if voltage >= self.MAX_VOLTAGE:
            return 100
        elif voltage <= self.MIN_VOLTAGE:
            return 0
        else:
            voltage_range = self.MAX_VOLTAGE - self.MIN_VOLTAGE
            percentage = ((voltage - self.MIN_VOLTAGE) /
                          voltage_range) * 100
            return max(0, min(100, int(round(percentage))))

    def get_battery_status_string(self, percentage: int) -> str:
        """Get battery status string based on percentage."""
        return '{0}%'.format(percentage)

    def start_receiving(self):
        """Start polling for battery ADC in a separate thread."""
        if self.receiving:
            return True

        self.receiving = True
        self.receive_thread = threading.Thread(target=self._poll_battery,
                                               daemon=True)
        self.receive_thread.start()
        print(f'Started battery polling (every {self.poll_interval}s)')
        return True

    def stop_receiving(self):
        """Stop polling for battery ADC."""
        self.receiving = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            print('Stopped battery polling')

    def read_battery_adc(self) -> int:
        """Read battery ADC value from holding register."""
        try:
            with self.instrument_lock:
                return self.instrument.read_register(self.HREG_BATTERY_ADC,
                                                     functioncode=3)
        except minimalmodbus.NoResponseError:
            return None
        except Exception:
            return None

    def _poll_battery(self):
        """Poll for battery ADC readings in background thread."""
        while self.receiving:
            try:
                # Skip update if navigating
                if self.is_navigating:
                    time.sleep(self.poll_interval)
                    continue

                # Read battery ADC value
                battery_adc = self.read_battery_adc()

                if battery_adc is not None and 0 < battery_adc <= self.ADC_MAX:
                    # Add to sample buffer
                    self.sample_buffer.append(battery_adc)

                    # Need at least 3 samples for median
                    if len(self.sample_buffer) >= 3:
                        # Calculate median of recent samples
                        median_adc = statistics.median(self.sample_buffer)
                        voltage = self.adc_to_voltage(median_adc)
                        percentage = self.voltage_to_percentage(voltage)

                        # Only emit if percentage changed
                        if percentage != self.current_percentage:
                            self.current_percentage = percentage
                            status_string = self.get_battery_status_string(
                                percentage)

                            self.battery_status_update.emit(int(median_adc))
                            self.battery_percentage_update.emit(
                                percentage, status_string)

                            print(f'Battery: {percentage}% '
                                  f'({voltage:.3f}V, ADC: {int(median_adc)})')

            except Exception as e:
                if self.receiving:
                    print(f'Battery: Error: {e}')

            # Sleep for poll interval
            time.sleep(self.poll_interval)

    def get_connection_status(self) -> dict:
        """Get current connection status for debugging."""
        return {
            'connected': self.instrument is not None,
            'receiving': self.receiving,
            'poll_interval': self.poll_interval,
            'thread_alive': (self.receive_thread.is_alive()
                             if self.receive_thread else False),
            'is_navigating': self.is_navigating,
            'current_percentage': self.current_percentage
        }
