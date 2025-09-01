#!/usr/bin/env python3

import socket
import struct
import threading

from PyQt5.QtCore import pyqtSignal, QObject


class CANBatteryReceiver(QObject):
    """Receives battery status messages from CAN bus.

    Frame ID: 42 (0x2A)
    Data: 2 bytes representing battery ADC reading (0-1023)
    """

    battery_status_update = pyqtSignal(int)  # Raw ADC value (0-1023)
    battery_percentage_update = pyqtSignal(int, str)  # Battery percentage (0-100) and status string

    def __init__(self, can_interface: str = 'can0'):
        super().__init__()
        self.can_interface = can_interface
        self.socket_fd = None
        self.receiving = False
        self.receive_thread = None

        # Battery frame ID
        self.BATTERY_FRAME_ID = 0x42  # Frame ID 42
        
        # Battery voltage constants for 10S Li-ion pack
        self.MAX_VOLTAGE = 42.0  # 100% - 1023 ADC value
        self.MIN_VOLTAGE = 32.0  # 0% - lowest safe voltage
        self.NOMINAL_VOLTAGE = 36.0  # 10S * 3.6V nominal
        self.WARNING_PERCENTAGE = 10  # Warning threshold

    def adc_to_voltage(self, adc_value: int) -> float:
        """Convert ADC value (0-1023) to voltage (V)."""
        return (adc_value / 1023.0) * self.MAX_VOLTAGE
    
    def voltage_to_percentage(self, voltage: float) -> int:
        """Convert voltage to battery percentage (0-100)."""
        if voltage >= self.MAX_VOLTAGE:
            return 100
        elif voltage <= self.MIN_VOLTAGE:
            return 0
        else:
            # Linear interpolation between min and max voltage
            percentage = ((voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100
            return max(0, min(100, int(round(percentage))))
    
    def get_battery_status_string(self, percentage: int, voltage: float) -> str:
        """Get battery status string based on percentage."""
        if percentage <= self.WARNING_PERCENTAGE:
            return f"{percentage}% WARNING"
        else:
            return f"{percentage}%"

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
        """Receive and parse CAN messages in background thread."""
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
                        # Convert ADC to voltage and percentage
                        voltage = self.adc_to_voltage(battery_adc)
                        percentage = self.voltage_to_percentage(voltage)
                        status_string = self.get_battery_status_string(percentage, voltage)
                        
                        
                        # Emit both raw ADC and processed percentage/status
                        self.battery_status_update.emit(battery_adc)
                        self.battery_percentage_update.emit(percentage, status_string)
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
