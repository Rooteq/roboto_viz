#!/usr/bin/env python3

import minimalmodbus
import serial
import threading
import time
from typing import Dict
from PyQt5.QtCore import QObject, pyqtSlot
from enum import IntEnum


class ModbusCoil(IntEnum):
    """Modbus coil addresses for LED and buzzer control."""

    GREEN_LED = 0    # Coil 0 - Good status (green LED)
    ORANGE_LED = 1   # Coil 1 - Warning status (orange LED)
    RED_LED = 2      # Coil 2 - Error status (red LED)
    BUZZER = 3       # Coil 3 - Buzzer control


class StatusLevel(IntEnum):
    """Status severity levels."""

    OK = 0
    WARNING = 1
    ERROR = 2


class CANStatusManager(QObject):
    """
    Manages Modbus RTU transmission for robot status signals.

    Sends status updates as Modbus coil writes when status changes occur.
    Note: Class name kept as CANStatusManager for backward compatibility.
    """

    def __init__(self, port: str = "/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if00-port0", slave_id: int = 1):
        super().__init__()
        self.port = port
        self.slave_id = slave_id
        self.instrument = None
        self.instrument_lock = threading.Lock()  # Shared lock for Modbus access
        self.last_status_cache: Dict[str, tuple] = {}
        self.battery_warning_active = False
        self.last_battery_warning_state = None
        self.is_navigating = False
        self.navigation_preparation_active = False
        self.last_buzzer_state = None
        self.modbus_paused = False

        # Continuous sending state
        self.current_led_coil = None
        self.current_buzzer_state = None
        self._continuous_timer = None

        # Rate limiting for collision detection
        self.last_collision_state = None
        self.last_collision_time = 0
        self.COLLISION_UPDATE_INTERVAL = 0.5  # Only update every 500ms

        # Status level mapping for different status strings
        self.status_level_map = {
            # OK states
            'idle': StatusLevel.OK,
            'at base': StatusLevel.OK,
            'at destination': StatusLevel.OK,
            'docked': StatusLevel.OK,
            'undocked': StatusLevel.OK,
            'connected': StatusLevel.OK,
            'available': StatusLevel.OK,

            # Warning states
            'warning': StatusLevel.WARNING,
            'low battery': StatusLevel.WARNING,
            'obstacle detected': StatusLevel.WARNING,
            'wykryto przeszkodę': StatusLevel.WARNING,
            'oczekiwanie na sygnał': StatusLevel.WARNING,

            # Error states
            'failed': StatusLevel.ERROR,
            'error': StatusLevel.ERROR,
            'błąd': StatusLevel.ERROR,
            'connection lost': StatusLevel.ERROR,
            'navigation error': StatusLevel.ERROR,
            'navigation failed': StatusLevel.ERROR,
            'dock failed': StatusLevel.ERROR,
            'undock failed': StatusLevel.ERROR,
            'not available': StatusLevel.ERROR,
        }

    def connect_can(self) -> bool:
        """
        Connect to Modbus RTU interface.

        Returns True if successful, False otherwise.
        Note: Method name kept as connect_can for backward compatibility.
        """
        try:
            self.instrument = minimalmodbus.Instrument(self.port, self.slave_id)
            self.instrument.serial.baudrate = 9600
            self.instrument.serial.parity = serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.bytesize = 8
            self.instrument.serial.timeout = 0.3  # Reduced from 1s to 0.3s
            self.instrument.mode = minimalmodbus.MODE_RTU

            # Test connection by turning off all outputs
            self._turn_off_all_outputs()

            print(f"Modbus Status Manager connected to {self.port} (slave {self.slave_id})")
            return True

        except Exception as e:
            print(f"Failed to connect to Modbus interface {self.port}: {e}")
            self.instrument = None
            return False

    def disconnect_can(self):
        """Disconnect from Modbus interface.

        Robust shutdown - never blocks, always cleans up.
        """
        self._stop_continuous_sending()
        if self.instrument:
            try:
                # Try to turn off all outputs (timeout protected)
                # Don't block if this fails
                try:
                    self._turn_off_all_outputs()
                except Exception:
                    pass  # Ignore errors during shutdown

                # Close serial port
                if hasattr(self.instrument, 'serial') and self.instrument.serial:
                    try:
                        self.instrument.serial.close()
                    except Exception:
                        pass  # Ignore close errors

                print('Modbus Status Manager disconnected')
            except Exception as e:
                print(f'Error disconnecting from Modbus: {e}')
            finally:
                self.instrument = None

    def _turn_off_all_outputs(self):
        """Turn off all LEDs and buzzer."""
        if not self.instrument:
            return
        try:
            with self.instrument_lock:
                for coil in ModbusCoil:
                    self.instrument.write_bit(int(coil), 0, functioncode=5)
        except Exception as e:
            print(f"Error turning off outputs: {e}")

    def _start_continuous_sending(self):
        """Start continuous 0.5Hz sending of current LED and buzzer states.

        Reduced from 1Hz to 0.5Hz (2s interval) to reduce Modbus bus load.
        """
        if self._continuous_timer is None:
            from PyQt5.QtCore import QTimer
            self._continuous_timer = QTimer()
            self._continuous_timer.timeout.connect(self._send_continuous_messages)
            self._continuous_timer.start(2000)  # 2s interval (was 1s)

    def _stop_continuous_sending(self):
        """Stop continuous sending timer."""
        if self._continuous_timer is not None:
            self._continuous_timer.stop()
            self._continuous_timer = None

    def _send_continuous_messages(self):
        """Send current LED and buzzer messages continuously at 0.5Hz.

        Skips sending if Modbus is paused (e.g., during map load/nav start).
        """
        if self.modbus_paused:
            return  # Don't send when paused

        if self.current_led_coil is not None:
            self._send_led_message(self.current_led_coil)

        if self.current_buzzer_state is not None:
            self._send_buzzer_message(self.current_buzzer_state)

    def _set_led_state_and_send(self, led_coil: ModbusCoil):
        """Set new LED state and start continuous sending."""
        self.current_led_coil = led_coil
        self._send_led_message(led_coil)
        self._start_continuous_sending()

    def _set_buzzer_state_and_send(self, buzzer_on: bool):
        """Set new buzzer state and start continuous sending."""
        self.current_buzzer_state = buzzer_on
        self._send_buzzer_message(buzzer_on)
        self._start_continuous_sending()

    def _get_status_level(self, status_text: str) -> StatusLevel:
        """Determine status level based on status text."""
        status_lower = status_text.lower().strip()

        if status_lower in self.status_level_map:
            return self.status_level_map[status_lower]

        for key, level in self.status_level_map.items():
            if key in status_lower:
                return level

        if any(word in status_lower for word in
               ['nav', 'navigating', 'executing', 'docking', 'undocking', 'manual']):
            return StatusLevel.OK
        elif any(word in status_lower for word in ['warn', 'low', 'obstacle']):
            return StatusLevel.WARNING
        elif any(word in status_lower for word in ['fail', 'error', 'lost', 'cancel']):
            return StatusLevel.ERROR
        else:
            return StatusLevel.OK

    def _send_led_message(self, led_coil: ModbusCoil):
        """
        Send Modbus message for LED control.

        Turns on specified LED and turns off other LEDs.
        """
        if self.modbus_paused:
            return True

        if not self.instrument:
            return False

        try:
            with self.instrument_lock:
                # Turn off all LEDs first, then turn on the desired one
                for coil in [ModbusCoil.GREEN_LED, ModbusCoil.ORANGE_LED, ModbusCoil.RED_LED]:
                    if coil == led_coil:
                        self.instrument.write_bit(int(coil), 1, functioncode=5)
                    else:
                        self.instrument.write_bit(int(coil), 0, functioncode=5)
            return True

        except Exception as e:
            print(f"Modbus ERROR: Failed to send LED message: {e}")
            return False

    def _send_buzzer_message(self, buzzer_on: bool):
        """Send Modbus message for buzzer control."""
        if self.modbus_paused:
            return True

        if not self.instrument:
            return False

        try:
            with self.instrument_lock:
                self.instrument.write_bit(
                    int(ModbusCoil.BUZZER), 1 if buzzer_on else 0, functioncode=5)
            return True

        except Exception as e:
            print(f"Modbus ERROR: Failed to send buzzer message: {e}")
            return False

    def send_buzzer_status(self, collision_detected: bool):
        """
        Send buzzer control message based on collision detection status.

        Only sends buzzer ON when both collision is detected AND robot is navigating.
        """
        if not self.instrument:
            return False

        if self.navigation_preparation_active:
            return True

        should_buzz = collision_detected and self.is_navigating

        if self.last_buzzer_state != should_buzz:
            self.last_buzzer_state = should_buzz
            self._set_buzzer_state_and_send(should_buzz)
            return True

        return True

    def _should_send_led(self, status_level: StatusLevel) -> bool:
        """Check if LED status should be sent (only send on change)."""
        cache_key = "led_status"

        if cache_key not in self.last_status_cache:
            self.last_status_cache[cache_key] = status_level
            return True

        if self.last_status_cache[cache_key] != status_level:
            self.last_status_cache[cache_key] = status_level
            return True

        return False

    def _should_block_ok_status(self, status_level: StatusLevel) -> bool:
        """Check if OK status should be blocked due to battery warning."""
        return status_level == StatusLevel.OK and self.battery_warning_active

    def send_led_status_if_changed(self, status_text: str):
        """Send LED control message only if status level has changed."""
        status_level = self._get_status_level(status_text)

        if self._should_block_ok_status(status_level):
            return

        if self._should_send_led(status_level):
            led_mapping = {
                StatusLevel.OK: ModbusCoil.GREEN_LED,
                StatusLevel.WARNING: ModbusCoil.ORANGE_LED,
                StatusLevel.ERROR: ModbusCoil.RED_LED
            }

            led_coil = led_mapping[status_level]
            self._set_led_state_and_send(led_coil)

    @pyqtSlot(str)
    def handle_robot_status(self, status: str):
        """Handle robot status updates."""
        self.send_led_status_if_changed(status)

    @pyqtSlot()
    def handle_navigation_actually_started(self):
        """Handle when navigation actually starts."""
        self.is_navigating = True

    @pyqtSlot(str)
    def handle_navigation_status(self, status: str):
        """Handle navigation status updates."""
        nav_inactive_keywords = [
            "zatrzymany", "stopped", "na miejscu", "w bazie",
            "bezczynny", "idle", "błąd", "failed", "error"]

        status_lower = status.lower()
        if any(keyword in status_lower for keyword in nav_inactive_keywords):
            self.is_navigating = False
            if self.instrument and not self.navigation_preparation_active:
                if self.last_buzzer_state:
                    self.last_buzzer_state = False
                    self._set_buzzer_state_and_send(False)

        self.send_led_status_if_changed(status)

    @pyqtSlot(str)
    def handle_docking_status(self, status: str):
        """Handle docking status updates."""
        self.send_led_status_if_changed(status)

    @pyqtSlot(str)
    def handle_manual_status(self, status: str):
        """Handle manual control status updates."""
        self.send_led_status_if_changed(status)

    @pyqtSlot(str)
    def handle_battery_status(self, status: str):
        """Handle battery status updates."""
        is_battery_warning = "WARNING" in status.upper() or "LOW BATTERY" in status.upper()

        if self.last_battery_warning_state != is_battery_warning:
            self.last_battery_warning_state = is_battery_warning
            self.battery_warning_active = is_battery_warning

            if is_battery_warning:
                self._set_led_state_and_send(ModbusCoil.ORANGE_LED)
            else:
                self._set_led_state_and_send(ModbusCoil.GREEN_LED)

    @pyqtSlot(str)
    def handle_plan_status(self, status: str):
        """Handle plan execution status updates."""
        self.send_led_status_if_changed(status)

    @pyqtSlot(str)
    def handle_wait_action_status(self, status: str):
        """Handle wait action status."""
        if "waiting for" in status.lower():
            self._set_led_state_and_send(ModbusCoil.ORANGE_LED)
        elif "received" in status.lower():
            if not self.battery_warning_active:
                self._set_led_state_and_send(ModbusCoil.GREEN_LED)

    def add_status_mapping(self, status_text: str, level: StatusLevel):
        """Add or update status level mapping."""
        self.status_level_map[status_text.lower().strip()] = level

    def send_stop_ok_message(self):
        """Send OK or WARNING message when STOP is pressed."""
        if self.last_buzzer_state:
            self.last_buzzer_state = False
            self._set_buzzer_state_and_send(False)

        if self.battery_warning_active:
            self._set_led_state_and_send(ModbusCoil.ORANGE_LED)
            return True

        self._set_led_state_and_send(ModbusCoil.GREEN_LED)
        return True

    def send_navigation_preparation_message(self):
        """Send buzzer ON and warning LED for navigation preparation."""
        self.navigation_preparation_active = True
        self.last_buzzer_state = True

        self.current_led_coil = ModbusCoil.ORANGE_LED
        self.current_buzzer_state = True

        self._send_led_message(ModbusCoil.ORANGE_LED)
        self._send_buzzer_message(True)

        self._start_continuous_sending()
        return True

    def stop_navigation_preparation(self):
        """Stop navigation preparation."""
        if not self.navigation_preparation_active:
            return

        self.navigation_preparation_active = False

        self.last_buzzer_state = False
        self.current_buzzer_state = False
        self._send_buzzer_message(False)

        if self.battery_warning_active:
            self.current_led_coil = ModbusCoil.ORANGE_LED
            self._send_led_message(ModbusCoil.ORANGE_LED)
        else:
            self.current_led_coil = ModbusCoil.GREEN_LED
            self._send_led_message(ModbusCoil.GREEN_LED)

    def send_navigation_start_ok_message(self):
        """Send OK or WARNING message when navigation starts."""
        self.stop_navigation_preparation()

    def get_status_info(self) -> Dict:
        """Get current status information for debugging."""
        return {
            'connected': self.instrument is not None,
            'port': self.port,
            'slave_id': self.slave_id,
            'last_led_status': self.last_status_cache.get('led_status', 'None'),
            'status_mappings_count': len(self.status_level_map),
            'coil_addresses': {
                'GREEN_LED': int(ModbusCoil.GREEN_LED),
                'ORANGE_LED': int(ModbusCoil.ORANGE_LED),
                'RED_LED': int(ModbusCoil.RED_LED),
                'BUZZER': int(ModbusCoil.BUZZER)
            }
        }

    @pyqtSlot(bool)
    def handle_collision_detection(self, collision_detected: bool):
        """Handle collision detection updates with rate limiting.

        Rate limited to max 2Hz to prevent Modbus bus flooding.
        Collision topics can publish at 10-30Hz which would overwhelm serial.
        """
        if not self.is_navigating or self.navigation_preparation_active:
            return

        import time
        current_time = time.time()

        # Rate limit: only update every 500ms AND when state changes
        state_changed = collision_detected != self.last_collision_state
        time_elapsed = current_time - self.last_collision_time
        should_update = state_changed and time_elapsed >= self.COLLISION_UPDATE_INTERVAL

        if not should_update:
            return

        # Update tracking
        self.last_collision_state = collision_detected
        self.last_collision_time = current_time

        # Send Modbus updates
        if collision_detected:
            if not self.last_buzzer_state:
                self.last_buzzer_state = True
                self._set_buzzer_state_and_send(True)
            self._set_led_state_and_send(ModbusCoil.ORANGE_LED)
        else:
            if self.last_buzzer_state:
                self.last_buzzer_state = False
                self._set_buzzer_state_and_send(False)
            if self.battery_warning_active:
                self._set_led_state_and_send(ModbusCoil.ORANGE_LED)
            else:
                self._set_led_state_and_send(ModbusCoil.GREEN_LED)

    def pause_can_messages(self):
        """Pause all Modbus message sending.

        Called during map loading and navigation start to free ROS2 resources.
        """
        self.modbus_paused = True
        print('Modbus: All operations paused')

    def resume_can_messages(self):
        """Resume Modbus message sending."""
        self.modbus_paused = False
        print('Modbus: All operations resumed')
