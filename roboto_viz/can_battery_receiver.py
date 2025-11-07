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

    Simple approach:
    - Only updates battery level when robot is NOT navigating
    - Uses median filter on recent samples
    - Ignores values below 125 ADC (~5.1V / 0% battery)
    - Ignores messages for 5s after detecting 2s+ message break
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
        self.IGNORE_DURATION = 10.0  # Ignore for 5 seconds after break

        # Update timing
        self.last_update_time = None
        self.MIN_UPDATE_INTERVAL = 5.0  # Update at most every 5 seconds

        # Startup
        self.initialized = False
        self.start_time = None

        # Battery voltage constants for 10S Li-ion pack
        self.MAX_VOLTAGE = 42.0  # 100% - 1023 ADC value
        self.MIN_VOLTAGE = 32.0  # 0% - lowest safe voltage
        self.NOMINAL_VOLTAGE = 36.0  # 10S * 3.6V nominal
        self.WARNING_PERCENTAGE = 10  # Warning threshold

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

    def connect_can(self) -> bool:
        """Connect to CAN interface for receiving messages."""
        try:
            # Create CAN socket
            self.socket_fd = socket.socket(socket.PF_CAN, socket.SOCK_RAW,
                                           socket.CAN_RAW)
            # Set up CAN filter to only receive battery status messages
            can_filter = struct.pack('=II', self.BATTERY_FRAME_ID, 0x7FF)
            self.socket_fd.setsockopt(socket.SOL_CAN_RAW,
                                      socket.CAN_RAW_FILTER, can_filter)

            # Bind to interface
            self.socket_fd.bind((self.can_interface,))

            print(f'CAN Battery Receiver connected to {self.can_interface}')
            return True

        except Exception as e:
            print(f'Failed to connect to CAN interface '
                  f'{self.can_interface}: {e}')
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
        self.start_time = time.time()
        self.receive_thread = threading.Thread(target=self._receive_messages,
                                               daemon=True)
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
        """Receive and parse CAN messages in background thread."""
        while self.receiving and self.socket_fd:
            try:
                # Set socket timeout to prevent infinite blocking
                self.socket_fd.settimeout(1.0)

                # Receive CAN frame
                frame, _ = self.socket_fd.recvfrom(16)

                # Parse CAN frame
                can_id, dlc, data = struct.unpack('=IB3x8s', frame)

                # Check if this is a battery status frame
                if can_id == self.BATTERY_FRAME_ID and dlc == 2:
                    # Extract 2 bytes of battery data
                    high_byte = data[0]
                    low_byte = data[1]

                    # Combine bytes to get ADC reading (0-1023)
                    battery_adc = (high_byte << 8) | low_byte

                    # Only accept values above threshold (125 ADC = ~5.1V)
                    # Below this indicates 0% battery or invalid reading
                    if 125 < battery_adc <= 1023:
                        current_time = time.time()

                        # Detect message break (>2 seconds gap)
                        if self.last_message_time is not None:
                            time_since_last = current_time - self.last_message_time
                            if time_since_last > self.MESSAGE_BREAK_THRESHOLD:
                                # Break detected - ignore for 5 seconds
                                self.ignore_until_time = (
                                    current_time + self.IGNORE_DURATION)
                                print(f'Battery: Message break detected '
                                      f'({time_since_last:.1f}s), '
                                      f'ignoring for 5 seconds')

                        self.last_message_time = current_time

                        # Check if we should ignore this message
                        if (self.ignore_until_time is not None and
                                current_time < self.ignore_until_time):
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
                                      f'({voltage:.1f}V)')
                            continue

                        # After initialization, only update when moving
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
                                      f'{new_percentage}% ({voltage:.1f}V)')
                else:
                    print(f'ADC below 125 received: {battery_adc}')

            except socket.timeout:
                # Normal timeout, continue receiving
                continue
            except Exception as e:
                if self.receiving:
                    print(f'Error receiving CAN battery message: {e}')
                break

    def get_connection_status(self) -> dict:
        """Get current connection status for debugging."""
        return {
            'connected': self.socket_fd is not None,
            'interface': self.can_interface,
            'receiving': self.receiving,
            'battery_frame_id': f'0x{self.BATTERY_FRAME_ID:02X}',
            'thread_alive': (self.receive_thread.is_alive()
                             if self.receive_thread else False),
            'is_navigating': self.is_navigating
        }
