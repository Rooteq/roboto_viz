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
    IMPORTANT: Uses shared instrument with thread lock to prevent collisions.

    Simple approach:
    - Only updates battery level when robot is NOT navigating
    - Uses median filter on recent samples
    - Ignores values below minimum threshold
    - Ignores messages for 10s after detecting 2s+ message break
    """

    battery_status_update = pyqtSignal(int)  # Filtered ADC (0-4095)
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
        self.sample_buffer = deque(maxlen=15)  # Keep 15 recent samples
        self.current_percentage = None  # Current displayed percentage
        self.current_voltage = None  # Current displayed voltage

        # Navigation state
        self.is_navigating = False  # Whether robot is in navigation action

        # Message timing for detecting breaks
        self.last_message_time = None
        self.ignore_until_time = None  # Ignore messages until this time
        self.MESSAGE_BREAK_THRESHOLD = 2.0  # 2 seconds break detection
        self.IGNORE_DURATION = 10.0  # Ignore for 10 seconds after break

        # Update timing
        self.last_update_time = None
        self.MIN_UPDATE_INTERVAL = 5.0  # Update at most every 5 seconds

        # Startup
        self.initialized = False
        self.start_time = None

        # Battery voltage constants for 10S Li-ion pack
        # ESP32 ADC is 12-bit (0-4095) with 11db attenuation (~3.6V max)
        self.ADC_MAX = 4095.0  # 12-bit ADC
        self.ADC_VOLTAGE_MAX = 3.6  # ESP32 ADC_11db attenuation max voltage
        self.MAX_VOLTAGE = 42.0  # 100% - 10S Li-ion full charge
        self.MIN_VOLTAGE = 32.0  # 0% - lowest safe voltage
        self.NOMINAL_VOLTAGE = 36.0  # 10S * 3.6V nominal
        self.WARNING_PERCENTAGE = 10  # Warning threshold
        # Minimum ADC value to accept (~0.87V input, ~29V battery)
        self.MIN_ADC_THRESHOLD = 1000

    def set_navigation_state(self, is_navigating: bool):
        """Update navigation state.

        Args:
            is_navigating: True if robot is currently navigating
        """
        self.is_navigating = is_navigating
        if is_navigating:
            print('Battery: Navigation started - updates paused')
        else:
            print('Battery: Navigation ended - updates will resume')

    def adc_to_voltage(self, adc_value: int) -> float:
        """Convert 12-bit ADC value to battery voltage (V).

        ESP32 with ADC_11db attenuation measures up to ~3.6V.
        Battery voltage is scaled down through voltage divider.
        """
        # Convert ADC to input voltage
        input_voltage = (adc_value / self.ADC_MAX) * self.ADC_VOLTAGE_MAX
        # Scale up to battery voltage (assuming appropriate voltage divider)
        # For 42V max battery and 3.6V max ADC: divider ratio ~11.67
        battery_voltage = (input_voltage *
                           (self.MAX_VOLTAGE / self.ADC_VOLTAGE_MAX))
        return battery_voltage

    def voltage_to_percentage(self, voltage: float) -> int:
        """Convert voltage to battery percentage (0-100).

        Scales so that 90% becomes 100%, 0% stays 0%.
        """
        if voltage >= self.MAX_VOLTAGE:
            raw_percentage = 100
        elif voltage <= self.MIN_VOLTAGE:
            return 0
        else:
            # Linear interpolation between min and max voltage
            voltage_range = self.MAX_VOLTAGE - self.MIN_VOLTAGE
            raw_percentage = ((voltage - self.MIN_VOLTAGE) /
                              voltage_range) * 100

        # Scale so that 90% becomes 100%, 0% stays 0%
        # Formula: scaled = (raw / 90) * 100
        scaled_percentage = (raw_percentage / 90.0) * 100.0
        return max(0, min(100, int(round(scaled_percentage))))

    def get_battery_status_string(
            self, percentage: int, voltage: float) -> str:
        """Get battery status string based on percentage."""
        return '{0}%'.format(percentage)

    def should_update_battery(self) -> bool:
        """Check if we should update the battery percentage.

        Returns True if:
        - Robot is NOT navigating, AND
        - Enough time has passed since last update
        """
        # Don't update when navigating
        if self.is_navigating:
            return False

        # Always allow first update after initialization
        if self.last_update_time is None:
            return True

        # Check if enough time has passed
        time_since_update = time.time() - self.last_update_time
        return time_since_update >= self.MIN_UPDATE_INTERVAL

    def start_receiving(self):
        """Start polling for battery ADC in a separate thread."""
        if self.receiving:
            return True  # Already receiving

        self.receiving = True
        self.start_time = time.time()
        self.receive_thread = threading.Thread(target=self._poll_battery,
                                               daemon=True)
        self.receive_thread.start()
        print('Started polling for Modbus battery ADC '
              f'(every {self.poll_interval}s)')
        return True

    def stop_receiving(self):
        """Stop polling for battery ADC."""
        self.receiving = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            print('Stopped polling for Modbus battery ADC')

    def read_battery_adc(self) -> int:
        """Read battery ADC value from holding register."""
        try:
            with self.instrument_lock:
                # Function code 3: Read Holding Registers
                return self.instrument.read_register(self.HREG_BATTERY_ADC,
                                                     functioncode=3)
        except minimalmodbus.NoResponseError:
            # Suppress verbose no-response warnings
            return None
        except Exception as e:
            if self.receiving:
                print(f'Battery: Read error: {e}')
            return None

    def _poll_battery(self):
        """Poll for battery ADC readings in background thread."""
        while self.receiving:
            try:
                # Read battery ADC value
                battery_adc = self.read_battery_adc()

                if (battery_adc is not None and
                        self.MIN_ADC_THRESHOLD < battery_adc <= self.ADC_MAX):
                    current_time = time.time()

                    # Detect message break (>2 seconds gap)
                    if self.last_message_time is not None:
                        time_since_last = current_time - self.last_message_time
                        if time_since_last > self.MESSAGE_BREAK_THRESHOLD:
                            # Break detected - ignore for 10 seconds
                            self.ignore_until_time = (
                                current_time + self.IGNORE_DURATION)
                            print(f'Battery: Message break detected '
                                  f'({time_since_last:.1f}s), '
                                  f'ignoring for {self.IGNORE_DURATION}s')

                    self.last_message_time = current_time

                    # Check if we should ignore this message
                    if (self.ignore_until_time is not None and
                            current_time < self.ignore_until_time):
                        time.sleep(self.poll_interval)
                        continue  # Ignore message

                    # Add to sample buffer
                    self.sample_buffer.append(battery_adc)

                    # Initial startup - emit first value after 1 second
                    if not self.initialized:
                        if (self.start_time and
                                time.time() - self.start_time >= 1.0 and
                                len(self.sample_buffer) >= 5):
                            # Get median of initial samples
                            median_adc = statistics.median(
                                self.sample_buffer)
                            voltage = self.adc_to_voltage(median_adc)
                            percentage = self.voltage_to_percentage(
                                voltage)
                            status_string = self.get_battery_status_string(
                                percentage, voltage)

                            # Store and emit initial values
                            self.current_percentage = percentage
                            self.current_voltage = voltage
                            self.battery_status_update.emit(
                                int(median_adc))
                            self.battery_percentage_update.emit(
                                percentage, status_string)
                            self.initialized = True
                            self.last_update_time = time.time()
                            print(f'Battery initialized: {percentage}% '
                                  f'({voltage:.1f}V, '
                                  f'ADC: {int(median_adc)})')
                        time.sleep(self.poll_interval)
                        continue

                    # After initialization, only update when not navigating
                    if (self.should_update_battery() and
                            len(self.sample_buffer) >= 5):
                        # Calculate median of recent samples
                        median_adc = statistics.median(self.sample_buffer)
                        voltage = self.adc_to_voltage(median_adc)
                        new_percentage = self.voltage_to_percentage(
                            voltage)

                        # Only emit if percentage actually changed
                        if new_percentage != self.current_percentage:
                            status_string = self.get_battery_status_string(
                                new_percentage, voltage)

                            self.current_percentage = new_percentage
                            self.current_voltage = voltage
                            self.battery_status_update.emit(
                                int(median_adc))
                            self.battery_percentage_update.emit(
                                new_percentage, status_string)
                            self.last_update_time = time.time()
                            print(f'Battery updated: '
                                  f'{new_percentage}% ({voltage:.1f}V, '
                                  f'ADC: {int(median_adc)})')
                elif battery_adc is not None:
                    pass  # Suppress "ADC below threshold" spam

            except Exception as e:
                if self.receiving:
                    print(f'Battery: Polling error: {e}')

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
            'current_percentage': self.current_percentage,
            'current_voltage': self.current_voltage
        }
