#!/usr/bin/env python3

import socket
import struct
from typing import Dict
from PyQt5.QtCore import QObject, pyqtSlot
from enum import IntEnum


class CANLEDType(IntEnum):
    """CAN message IDs for RGB status LED control"""
    GREEN_LED = 0x201   # Good status (green LED)
    ORANGE_LED = 0x202  # Warning status (orange LED) 
    RED_LED = 0x203     # Error status (red LED)


class CANBuzzerType(IntEnum):
    """CAN message IDs for buzzer control"""
    BUZZER_ON = 0x204   # Buzzer on (collision detected)
    BUZZER_OFF = 0x205  # Buzzer off (no collision)


class StatusLevel(IntEnum):
    """Status severity levels"""
    OK = 0
    WARNING = 1
    ERROR = 2


class CANStatusManager(QObject):
    """
    Manages CAN message transmission for robot status signals.
    Sends status updates as CAN messages when status changes occur.
    """

    def __init__(self, can_interface: str = "can0"):
        super().__init__()
        self.can_interface = can_interface
        self.socket_fd = None
        self.last_status_cache: Dict[str, tuple] = {}
        self.battery_warning_active = False  # Track if battery is in warning state
        self.last_battery_warning_state = None  # Track battery warning state changes
        self.is_navigating = False  # Track if robot is currently navigating
        self.navigation_preparation_active = False  # Track if in 5s preparation phase
        
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
            'oczekiwanie na sygnał': StatusLevel.WARNING,
            
            # Error states
            'failed': StatusLevel.ERROR,
            'error': StatusLevel.ERROR,
            'connection lost': StatusLevel.ERROR,
            'navigation error': StatusLevel.ERROR,
            'navigation failed': StatusLevel.ERROR,
            'dock failed': StatusLevel.ERROR,
            'undock failed': StatusLevel.ERROR,
            'not available': StatusLevel.ERROR,
        }
        
    def connect_can(self) -> bool:
        """
        Connect to CAN interface
        Returns True if successful, False otherwise
        """
        try:
            # Create CAN socket
            self.socket_fd = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            
            # Bind to interface
            self.socket_fd.bind((self.can_interface,))
            
            print(f"CAN Status Manager connected to {self.can_interface}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to CAN interface {self.can_interface}: {e}")
            self.socket_fd = None
            return False
            
    def disconnect_can(self):
        """Disconnect from CAN interface"""
        if self.socket_fd:
            try:
                self.socket_fd.close()
                print("CAN Status Manager disconnected")
            except Exception as e:
                print(f"Error disconnecting from CAN: {e}")
            finally:
                self.socket_fd = None
                
    def _get_status_level(self, status_text: str) -> StatusLevel:
        """
        Determine status level based on status text
        """
        status_lower = status_text.lower().strip()
        
        # Check for exact matches first
        if status_lower in self.status_level_map:
            return self.status_level_map[status_lower]
            
        # Check for partial matches
        for key, level in self.status_level_map.items():
            if key in status_lower:
                return level
                
        # Default logic based on common patterns
        if any(word in status_lower for word in ['nav', 'navigating', 'executing', 'docking', 'undocking', 'manual']):
            return StatusLevel.OK
        elif any(word in status_lower for word in ['warn', 'low', 'obstacle']):
            return StatusLevel.WARNING
        elif any(word in status_lower for word in ['fail', 'error', 'lost', 'cancel']):
            return StatusLevel.ERROR
        else:
            return StatusLevel.OK
            
    def _send_led_can_message(self, led_can_id: CANLEDType):
        """
        Send empty CAN message for LED control
        
        Message format: Empty frame (0 bytes) - just the CAN ID triggers LED
        """
        if not self.socket_fd:
            return False
            
        try:
            # Create empty CAN frame - just the ID is needed for LED control
            can_frame = struct.pack("=IB3x8s", 
                                  int(led_can_id),  # CAN ID
                                  0,                # Data length (0 bytes - empty frame)
                                  b'\x00' * 8)     # Empty data payload
            
            # Send frame
            self.socket_fd.send(can_frame)
            
            return True
            
        except Exception as e:
            print(f"Error sending CAN LED message: {e}")
            return False
            
    def _send_buzzer_can_message(self, buzzer_can_id: CANBuzzerType):
        """
        Send empty CAN message for buzzer control
        
        Message format: Empty frame (0 bytes) - just the CAN ID triggers buzzer
        """
        if not self.socket_fd:
            return False
            
        try:
            # Create empty CAN frame - just the ID is needed for buzzer control
            can_frame = struct.pack("=IB3x8s", 
                                  int(buzzer_can_id),  # CAN ID
                                  0,                   # Data length (0 bytes - empty frame)
                                  b'\x00' * 8)        # Empty data payload
            
            # Send frame
            self.socket_fd.send(can_frame)
            
            return True
            
        except Exception as e:
            print(f"Error sending CAN buzzer message: {e}")
            return False

    def send_buzzer_status(self, collision_detected: bool):
        """
        Send buzzer control message based on collision detection status.
        Only sends buzzer ON when both collision is detected AND robot is navigating.
        Always sends buzzer OFF when not navigating or no collision.
        Does NOT interfere with navigation preparation buzzer.
        """
        if not self.socket_fd:
            return False
        
        # Don't interfere with navigation preparation buzzer
        if self.navigation_preparation_active:
            return True
            
        # Only turn buzzer ON if collision detected AND robot is navigating
        should_buzz = collision_detected and self.is_navigating
        buzzer_id = CANBuzzerType.BUZZER_ON if should_buzz else CANBuzzerType.BUZZER_OFF
        return self._send_buzzer_can_message(buzzer_id)

    def _should_send_led(self, status_level: StatusLevel) -> bool:
        """
        Check if LED status should be sent (only send on change)
        """
        cache_key = "led_status"
        
        if cache_key not in self.last_status_cache:
            self.last_status_cache[cache_key] = status_level
            return True
            
        if self.last_status_cache[cache_key] != status_level:
            self.last_status_cache[cache_key] = status_level
            return True
            
        return False
    
    def _should_block_ok_status(self, status_level: StatusLevel) -> bool:
        """Check if OK status should be blocked due to battery warning"""
        return status_level == StatusLevel.OK and self.battery_warning_active
        
    def send_led_status_if_changed(self, status_text: str):
        """
        Send LED control message only if status level has changed
        """
        if not self.socket_fd:
            return
            
        status_level = self._get_status_level(status_text)
        
        # Block OK status if battery is in warning state
        if self._should_block_ok_status(status_level):
            return
        
        if self._should_send_led(status_level):
            # Map status level to LED CAN ID
            led_mapping = {
                StatusLevel.OK: CANLEDType.GREEN_LED,
                StatusLevel.WARNING: CANLEDType.ORANGE_LED,
                StatusLevel.ERROR: CANLEDType.RED_LED
            }
            
            led_can_id = led_mapping[status_level]
            self._send_led_can_message(led_can_id)
            
    # PyQt slots for connecting to existing status signals
    @pyqtSlot(str)
    def handle_robot_status(self, status: str):
        """Handle robot status updates"""
        self.send_led_status_if_changed(status)
        
    @pyqtSlot(str) 
    def handle_navigation_status(self, status: str):
        """Handle navigation status updates"""
        
        # Update navigation state based on status
        nav_active_keywords = ["nav to", "navigating", "nawigacja"]
        nav_inactive_keywords = ["zatrzymany", "stopped", "na miejscu", "w bazie", "bezczynny", "idle", "błąd", "failed", "error"]
        
        status_lower = status.lower()
        if any(keyword in status_lower for keyword in nav_active_keywords):
            self.is_navigating = True
        elif any(keyword in status_lower for keyword in nav_inactive_keywords):
            self.is_navigating = False
            # When navigation stops, send buzzer OFF (unless in preparation phase)
            if self.socket_fd and not self.navigation_preparation_active:
                self._send_buzzer_can_message(CANBuzzerType.BUZZER_OFF)
        
        self.send_led_status_if_changed(status)
        
    @pyqtSlot(str)
    def handle_docking_status(self, status: str):
        """Handle docking status updates"""
        self.send_led_status_if_changed(status)
        
    @pyqtSlot(str)
    def handle_manual_status(self, status: str):
        """Handle manual control status updates"""
        self.send_led_status_if_changed(status)
        
    @pyqtSlot(str)
    def handle_battery_status(self, status: str):
        """Handle battery status updates - only send on warning state changes"""
        # Check if this is a battery warning state
        is_battery_warning = "WARNING" in status.upper() or "LOW BATTERY" in status.upper()
        
        # Only send CAN message if battery warning state changed
        if self.last_battery_warning_state != is_battery_warning:
            self.last_battery_warning_state = is_battery_warning
            self.battery_warning_active = is_battery_warning
            
            # Send appropriate CAN message for battery state change
            if is_battery_warning:
                # Send WARNING LED for low battery
                self._send_led_can_message(CANLEDType.ORANGE_LED)
            else:
                # Battery recovered - send OK LED only if no other issues
                self._send_led_can_message(CANLEDType.GREEN_LED)
        
    @pyqtSlot(str)
    def handle_plan_status(self, status: str):
        """Handle plan execution status updates"""
        self.send_led_status_if_changed(status)
        
    @pyqtSlot(str)
    def handle_wait_action_status(self, status: str):
        """Handle wait action status - always send WARNING without blocking other messages"""
        if "waiting for" in status.lower():
            # Always send WARNING LED for wait actions, regardless of battery state
            self._send_led_can_message(CANLEDType.ORANGE_LED)
        elif "received" in status.lower():
            # Wait action completed - send OK LED only if no battery warning
            if not self.battery_warning_active:
                self._send_led_can_message(CANLEDType.GREEN_LED)
        
    def add_status_mapping(self, status_text: str, level: StatusLevel):
        """
        Add or update status level mapping
        Allows dynamic addition of new status mappings
        """
        self.status_level_map[status_text.lower().strip()] = level
    
    def send_stop_ok_message(self):
        """Send OK or WARNING message when STOP is pressed based on battery level"""
        if not self.socket_fd:
            return False
            
        if self.battery_warning_active:
            # Send WARNING LED instead of blocking the message
            success = self._send_led_can_message(CANLEDType.ORANGE_LED)
            return success
        
        # Send GREEN LED for successful stop
        success = self._send_led_can_message(CANLEDType.GREEN_LED)
        return success
    
    def send_navigation_preparation_message(self):
        """Send buzzer ON and warning LED for navigation preparation (during 5s countdown)"""
        if not self.socket_fd:
            return False
        
        # Set preparation phase flag to prevent interference
        self.navigation_preparation_active = True
        
        # Start beeping during preparation phase
        success_buzzer = self._send_buzzer_can_message(CANBuzzerType.BUZZER_ON)
        
        # Send WARNING LED for preparation phase
        success_led = self._send_led_can_message(CANLEDType.ORANGE_LED)
        
        # Start a timer to repeatedly send buzzer ON during preparation phase
        from PyQt5.QtCore import QTimer
        self._preparation_buzzer_timer = QTimer()
        self._preparation_buzzer_timer.timeout.connect(self._send_preparation_buzzer_repeat)
        self._preparation_buzzer_timer.start(500)  # Send buzzer ON every 500ms
        
        return success_buzzer and success_led
    
    def _send_preparation_buzzer_repeat(self):
        """Repeatedly send buzzer ON during navigation preparation to ensure it stays on"""
        if self.navigation_preparation_active and self.socket_fd:
            self._send_buzzer_can_message(CANBuzzerType.BUZZER_ON)
    
    def send_navigation_start_ok_message(self):
        """Send OK or WARNING message when navigation starts based on battery level"""
        if not self.socket_fd:
            return False
        
        # End preparation phase and stop the preparation buzzer
        self.navigation_preparation_active = False
        if hasattr(self, '_preparation_buzzer_timer'):
            self._preparation_buzzer_timer.stop()
        self._send_buzzer_can_message(CANBuzzerType.BUZZER_OFF)
        
        if self.battery_warning_active:
            # Send WARNING LED instead of blocking the message
            success = self._send_led_can_message(CANLEDType.ORANGE_LED)
            if success:
                # Send the same WARNING message again after 117ms for reliability
                from PyQt5.QtCore import QTimer
                QTimer.singleShot(117, self._send_navigation_start_warning_delayed)
            return success
        
        # Send GREEN LED for navigation start (first time)
        success = self._send_led_can_message(CANLEDType.GREEN_LED)
        if success:
            # Send the same message again after 117ms for reliability
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(117, self._send_navigation_start_ok_delayed)
        
        return success
    
    def _send_navigation_start_ok_delayed(self):
        """Send the navigation start OK message again after 117ms delay"""
        if self.socket_fd and not self.battery_warning_active:
            success = self._send_led_can_message(CANLEDType.GREEN_LED)
            if success:
                # Send third message after another 117ms (234ms total)
                from PyQt5.QtCore import QTimer
                QTimer.singleShot(117, self._send_navigation_start_ok_third)
    
    def _send_navigation_start_warning_delayed(self):
        """Send the navigation start WARNING message again after 117ms delay"""
        if self.socket_fd and self.battery_warning_active:
            success = self._send_led_can_message(CANLEDType.ORANGE_LED)
            if success:
                # Send third message after another 117ms (234ms total)
                from PyQt5.QtCore import QTimer
                QTimer.singleShot(117, self._send_navigation_start_warning_third)
    
    def _send_navigation_start_ok_third(self):
        """Send the navigation start OK message a third time after 234ms total delay"""
        if self.socket_fd and not self.battery_warning_active:
            self._send_led_can_message(CANLEDType.GREEN_LED)
                
    def _send_navigation_start_warning_third(self):
        """Send the navigation start WARNING message a third time after 234ms total delay"""
        if self.socket_fd and self.battery_warning_active:
            self._send_led_can_message(CANLEDType.ORANGE_LED)
        
    def get_status_info(self) -> Dict:
        """
        Get current status information for debugging
        """
        return {
            'connected': self.socket_fd is not None,
            'interface': self.can_interface,
            'last_led_status': self.last_status_cache.get('led_status', 'None'),
            'status_mappings_count': len(self.status_level_map),
            'led_can_ids': {
                'GREEN_LED': f"0x{int(CANLEDType.GREEN_LED):03X}",
                'ORANGE_LED': f"0x{int(CANLEDType.ORANGE_LED):03X}",
                'RED_LED': f"0x{int(CANLEDType.RED_LED):03X}"
            },
            'buzzer_can_ids': {
                'BUZZER_ON': f"0x{int(CANBuzzerType.BUZZER_ON):03X}",
                'BUZZER_OFF': f"0x{int(CANBuzzerType.BUZZER_OFF):03X}"
            }
        }
    
    @pyqtSlot(bool)
    def handle_collision_detection(self, collision_detected: bool):
        """Handle collision detection updates and control buzzer and LED status"""
        # Send buzzer control message (buzzer logic already handles navigation state)
        self.send_buzzer_status(collision_detected)

        # Only send collision-related LED status when robot is actively navigating
        if self.is_navigating:
            if collision_detected:
                # Always send WARNING LED for collision detection during navigation
                self._send_led_can_message(CANLEDType.ORANGE_LED)
            else:
                # No collision detected during navigation
                if self.battery_warning_active:
                    # Battery warning is active - keep sending WARNING LED
                    self._send_led_can_message(CANLEDType.ORANGE_LED)
                else:
                    # No collision and no battery warning during navigation - send OK LED
                    self._send_led_can_message(CANLEDType.GREEN_LED)