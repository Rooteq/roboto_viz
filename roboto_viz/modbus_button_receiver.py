#!/usr/bin/env python3
"""Modbus-based button receiver for ESP32 AMR Sound Viz Controller.

Polls for button clicks using Modbus RTU.
Replaces CAN-based signal receiver (can_signal_receiver.py)
Uses shared Modbus instrument to avoid serial port conflicts.
"""

import threading
import time

import minimalmodbus
from PyQt5.QtCore import pyqtSignal, QObject


class ModbusButtonReceiver(QObject):
    """Receives button click signals from ESP32 via Modbus RTU.

    Polls discrete input register for button click flag and clears it
    after detection to enable next click detection.
    IMPORTANT: Uses shared instrument with thread lock to prevent collisions.
    """

    signal_received = pyqtSignal()  # Signal for button click (wait action)

    # Modbus register addresses (must match relay_modbus.h)
    ISTS_BUTTON_CLICK = 0      # Discrete input address for button click flag
    COIL_BUTTON_CLEAR = 6      # Coil address to clear button click flag

    def __init__(self, shared_instrument, instrument_lock, poll_interval=1.0):
        """Initialize button receiver with shared Modbus instrument.

        Args:
            shared_instrument: Shared minimalmodbus.Instrument instance
            instrument_lock: threading.Lock for serializing access
            poll_interval: Polling interval in seconds (default: 1.0s)
        """
        super().__init__()
        self.instrument = shared_instrument
        self.instrument_lock = instrument_lock
        self.poll_interval = poll_interval
        self.receiving = False
        self.receive_thread = None
        self.click_count = 0
        self.paused = False  # Pause flag for critical operations

    def start_receiving(self):
        """Start polling for button clicks in a separate thread."""
        if self.receiving:
            return True  # Already receiving

        self.receiving = True
        self.receive_thread = threading.Thread(target=self._poll_button,
                                               daemon=True)
        self.receive_thread.start()
        print('Started polling for Modbus button clicks '
              f'(every {self.poll_interval}s)')
        return True

    def stop_receiving(self):
        """Stop polling for button clicks."""
        self.receiving = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            print('Stopped polling for Modbus button clicks')

    def read_button_click(self) -> bool:
        """Read button click flag from discrete input."""
        try:
            with self.instrument_lock:
                # Function code 2: Read Discrete Inputs
                result = self.instrument.read_bit(
                    self.ISTS_BUTTON_CLICK, functioncode=2)
                return result
        except minimalmodbus.NoResponseError:
            # Suppress verbose no-response warnings
            return False
        except Exception as e:
            if self.receiving:
                print(f'Button: Read error: {e}')
            return False

    def clear_button_click(self) -> None:
        """Clear button click flag by writing to coil."""
        try:
            with self.instrument_lock:
                # Function code 5: Write Single Coil
                self.instrument.write_bit(self.COIL_BUTTON_CLEAR, 1,
                                          functioncode=5)
        except minimalmodbus.NoResponseError:
            # Suppress verbose no-response warnings
            pass
        except Exception as e:
            if self.receiving:
                print(f'Button: Clear error: {e}')

    def _poll_button(self):
        """Poll for button clicks in background thread."""
        while self.receiving:
            try:
                # Skip polling if paused
                if self.paused:
                    time.sleep(self.poll_interval)
                    continue

                # Read button click flag
                if self.read_button_click():
                    self.click_count += 1
                    timestamp = time.strftime('%H:%M:%S')
                    print(f'[{timestamp}] Button clicked! '
                          f'(count: {self.click_count})')

                    # Emit signal for wait action completion
                    self.signal_received.emit()

                    # Clear the flag so we can detect the next click
                    self.clear_button_click()

            except Exception as e:
                if self.receiving:
                    print(f'Button: Polling error: {e}')

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
            'click_count': self.click_count
        }
