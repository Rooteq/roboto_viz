#!/usr/bin/env python3

import socket
import struct
import threading

from PyQt5.QtCore import pyqtSignal, QObject


class CANSignalReceiver(QObject):
    """Receives wait signal messages from CAN bus.

    Frame ID: 0x69
    Data: Any data triggers the signal (content ignored)
    """

    signal_received = pyqtSignal()  # Signal for wait action completion

    def __init__(self, can_interface: str = 'can0'):
        super().__init__()
        self.can_interface = can_interface
        self.socket_fd = None
        self.receiving = False
        self.receive_thread = None

        # Signal frame ID
        self.SIGNAL_FRAME_ID = 0x69  # Frame ID 0x69

    def connect_can(self) -> bool:
        """Connect to CAN interface for receiving messages."""
        try:
            # Create CAN socket
            self.socket_fd = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # Set up CAN filter to only receive signal messages (Frame ID 0x69)
            can_filter = struct.pack('=II', self.SIGNAL_FRAME_ID, 0x7FF)  # ID and mask
            self.socket_fd.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter)

            # Bind to interface
            self.socket_fd.bind((self.can_interface,))

            print(f'CAN Signal Receiver connected to {self.can_interface}')
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
                print('CAN Signal Receiver disconnected')
            except Exception as e:
                print(f'Error disconnecting from CAN: {e}')
            finally:
                self.socket_fd = None

    def start_receiving(self):
        """Start receiving signal messages in a separate thread."""
        if not self.socket_fd:
            if not self.connect_can():
                return False
        if self.receiving:
            return True  # Already receiving

        self.receiving = True
        self.receive_thread = threading.Thread(target=self._receive_messages, daemon=True)
        self.receive_thread.start()
        print('Started receiving CAN signal messages')
        return True

    def stop_receiving(self):
        """Stop receiving signal messages."""
        self.receiving = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
            print('Stopped receiving CAN signal messages')

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

                # Check if this is a signal frame (should be, due to filter)
                if can_id == self.SIGNAL_FRAME_ID:
                    print(f'CAN Signal: Received wait signal on ID 0x{can_id:02X}')
                    self.signal_received.emit()
                        
            except socket.timeout:
                # Normal timeout, continue receiving
                continue
            except Exception as e:
                if self.receiving:  # Only log errors if we're supposed to be receiving
                    print(f'Error receiving CAN signal message: {e}')
                break

    def get_connection_status(self) -> dict:
        """Get current connection status for debugging."""
        return {
            'connected': self.socket_fd is not None,
            'interface': self.can_interface,
            'receiving': self.receiving,
            'signal_frame_id': f'0x{self.SIGNAL_FRAME_ID:02X}',
            'thread_alive': self.receive_thread.is_alive() if self.receive_thread else False
        }