#!/usr/bin/env python3

from collections import deque
import socket
import statistics
import struct
import threading
import time

from PyQt5.QtCore import pyqtSignal, QObject


class CANBatteryReceiver(QObject):
    """Receives battery status messages from CAN bus.

    Frame ID: 42 (0x2A)
    Data: 2 bytes representing battery ADC reading (0-1023)

    Implements simple but effective filtering:
    1. Outlier rejection: ignores zeros and extreme jumps (>5V)
    2. Exponential Moving Average (EMA) for voltage smoothing
    3. Emission rate limiting to reduce UI update frequency
    """

    battery_status_update = pyqtSignal(int)  # Filtered ADC (0-1023)
    battery_percentage_update = pyqtSignal(int, str)  # % (0-100), str

    def __init__(self, can_interface: str = 'can0'):
        super().__init__()
        self.can_interface = can_interface
        self.socket_fd = None
        self.receiving = False
        self.receive_thread = None

        # Battery frame ID
        self.BATTERY_FRAME_ID = 0x42  # Frame ID 42

        # Filter variables
        self.sample_window = deque(maxlen=20)  # Rolling window for outlier detection
        self.voltage_ema = None  # Exponential moving average of voltage
        self.last_valid_adc = None  # Last known good ADC value

        # Filter parameters
        self.EMA_ALPHA = 0.05  # EMA smoothing (lower = more stable)
        self.MIN_SAMPLES_FOR_INIT = 5  # Minimum samples before first emission

        # Startup handling
        self.initial_value_set = False
        self.start_time = None
        self.INITIAL_DELAY = 1.0  # 1 second delay before first emission
        self.samples_received = 0

        # Emission rate limiting (emit every N samples to reduce UI updates)
        self.emission_counter = 0
        self.EMISSION_INTERVAL = 10  # Emit every 10 valid samples

        # Battery voltage constants for 10S Li-ion pack
        self.MAX_VOLTAGE = 42.0  # 100% - 1023 ADC value
        self.MIN_VOLTAGE = 32.0  # 0% - lowest safe voltage
        self.NOMINAL_VOLTAGE = 36.0  # 10S * 3.6V nominal
        self.WARNING_PERCENTAGE = 10  # Warning threshold

    def is_outlier(self, adc_value: int) -> bool:
        """Detect outliers - only reject zeros and extreme values.

        Returns True if the value is an outlier and should be rejected.
        """
        # Always reject zero values (invalid readings from microcontroller)
        if adc_value == 0:
            return True

        # Reject extremely low values (below 5% of range = ~2V)
        if adc_value < 50:
            return True

        # If we have previous valid readings, check for huge jumps
        if self.voltage_ema is not None and len(self.sample_window) >= 3:
            current_voltage = self.adc_to_voltage(adc_value)
            voltage_diff = abs(current_voltage - self.voltage_ema)

            # Reject only if jump is more than 5V (extremely unrealistic)
            if voltage_diff > 5.0:
                return True

        return False

    def apply_ema_filter(self, voltage: float) -> float:
        """Apply Exponential Moving Average filter.

        Smooths voltage readings over time with configurable smoothing factor.
        """
        if self.voltage_ema is None:
            self.voltage_ema = voltage
            return voltage

        # EMA formula: EMA_new = alpha * value + (1 - alpha) * EMA_old
        self.voltage_ema = (self.EMA_ALPHA * voltage +
                            (1 - self.EMA_ALPHA) * self.voltage_ema)
        return self.voltage_ema

    def adc_to_voltage(self, adc_value: int) -> float:
        """Convert ADC value (0-1023) to voltage (V)."""
        return (adc_value / 1023.0) * self.MAX_VOLTAGE

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

    def connect_can(self) -> bool:
        """Connect to CAN interface for receiving messages."""
        try:
            # Create CAN socket
            self.socket_fd = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # Set up CAN filter to only receive battery status messages (Frame ID 42)
            can_filter = struct.pack('=II', self.BATTERY_FRAME_ID, 0x7FF)  # ID and mask
            self.socket_fd.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter)

            # Bind to interface
            self.socket_fd.bind((self.can_interface,))

            print(f'CAN Battery Receiver connected to {self.can_interface}')
            return True

        except Exception as e:
            print(f'Failed to connect to CAN interface {self.can_interface}: {e}')
            self.socket_fd = None
            return False

    def disconnect_can(self):
        """Disconnect from CAN interface."""
        self.stop_receiving()
        if self.socket_fd:
            try:
                self.socket_fd.close()
                print('CAN Battery Receiver disconnected')
            except Exception as e:
                print(f'Error disconnecting from CAN: {e}')
            finally:
                self.socket_fd = None

    def start_receiving(self):
        """Start receiving battery status messages in a separate thread."""
        if not self.socket_fd:
            if not self.connect_can():
                return False
        if self.receiving:
            return True  # Already receiving

        self.receiving = True
        self.start_time = time.time()  # Record start time
        self.receive_thread = threading.Thread(target=self._receive_messages, daemon=True)
        self.receive_thread.start()
        print('Started receiving CAN battery messages')
        return True

    def stop_receiving(self):
        """Stop receiving battery status messages."""
        self.receiving = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
            print('Stopped receiving CAN battery messages')

    def _receive_messages(self):
        """Receive and parse CAN messages in background thread with multi-stage filtering."""
        while self.receiving and self.socket_fd:
            try:
                # Set socket timeout to prevent infinite blocking
                self.socket_fd.settimeout(1.0)

                # Receive CAN frame
                frame, _ = self.socket_fd.recvfrom(16)

                # Parse CAN frame: ID (4 bytes), DLC (1 byte), padding (3 bytes), Data (8 bytes)
                can_id, dlc, data = struct.unpack('=IB3x8s', frame)

                # Check if this is a battery status frame (should be, due to filter)
                if can_id == self.BATTERY_FRAME_ID and dlc == 2:
                    # Extract 2 bytes of battery data
                    high_byte = data[0]
                    low_byte = data[1]

                    # Combine bytes to get ADC reading (0-1023)
                    battery_adc = (high_byte << 8) | low_byte

                    # Ensure value is in expected range
                    if 0 <= battery_adc <= 1023:
                        # Stage 1: Outlier rejection (ignores zeros and spikes)
                        if self.is_outlier(battery_adc):
                            # Outlier detected - ignore this sample
                            if battery_adc == 0:
                                pass  # Silent for zero values
                            else:
                                print(f'Battery outlier rejected: {battery_adc}')
                            continue

                        # Add valid sample to rolling window
                        self.sample_window.append(battery_adc)
                        self.last_valid_adc = battery_adc
                        self.samples_received += 1

                        if not self.initial_value_set and self.samples_received <= 5:
                            print(f'Battery sample {self.samples_received}: '
                                  f'ADC={battery_adc}, '
                                  f'window_size={len(self.sample_window)}')

                        # Wait for initial delay period before first emission
                        if not self.initial_value_set:
                            if self.start_time is None:
                                continue  # Start time not set yet

                            current_time = time.time()
                            if current_time - self.start_time < self.INITIAL_DELAY:
                                continue  # Still in initial delay period

                            # Check if we have enough samples for initialization
                            if len(self.sample_window) < self.MIN_SAMPLES_FOR_INIT:
                                continue  # Not enough samples yet

                            # Initialize filter with median of current samples
                            init_adc = statistics.median(self.sample_window)
                            init_voltage = self.adc_to_voltage(init_adc)
                            self.voltage_ema = init_voltage
                            init_percentage = self.voltage_to_percentage(
                                init_voltage)
                            init_status = self.get_battery_status_string(
                                init_percentage, init_voltage)

                            # Emit initial value immediately
                            self.battery_status_update.emit(init_adc)
                            self.battery_percentage_update.emit(
                                init_percentage, init_status)
                            self.initial_value_set = True
                            print(f'Battery initialized: {init_percentage}% '
                                  f'({init_voltage:.1f}V)')
                            continue

                        # Skip processing if not initialized yet
                        if not self.initial_value_set:
                            continue

                        # Apply EMA filtering to smooth voltage
                        raw_voltage = self.adc_to_voltage(battery_adc)
                        filtered_voltage = self.apply_ema_filter(raw_voltage)

                        # Convert to percentage
                        percentage = self.voltage_to_percentage(filtered_voltage)

                        # Emission rate limiting (update UI less frequently)
                        self.emission_counter += 1
                        if self.emission_counter >= self.EMISSION_INTERVAL:
                            self.emission_counter = 0

                            # Emit signals with filtered values
                            filtered_adc = int((filtered_voltage /
                                                self.MAX_VOLTAGE) * 1023)
                            status_string = self.get_battery_status_string(
                                percentage, filtered_voltage)

                            self.battery_status_update.emit(filtered_adc)
                            self.battery_percentage_update.emit(
                                percentage, status_string)

                    else:
                        pass  # ADC value out of range

            except socket.timeout:
                # Normal timeout, continue receiving
                continue
            except Exception as e:
                if self.receiving:  # Only log errors if we're supposed to be receiving
                    print(f'Error receiving CAN battery message: {e}')
                break

    def get_connection_status(self) -> dict:
        """Get current connection status for debugging."""
        return {
            'connected': self.socket_fd is not None,
            'interface': self.can_interface,
            'receiving': self.receiving,
            'battery_frame_id': f'0x{self.BATTERY_FRAME_ID:02X}',
            'thread_alive': self.receive_thread.is_alive() if self.receive_thread else False
        }
