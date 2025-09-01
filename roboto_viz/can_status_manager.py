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
            'waiting for signal': StatusLevel.WARNING,
            
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
            
            print(f"CAN LED: Sent {led_can_id.name} (ID: 0x{int(led_can_id):03X})")
            return True
            
        except Exception as e:
            print(f"Error sending CAN LED message: {e}")
            return False
            
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
        
    def send_led_status_if_changed(self, status_text: str):
        """
        Send LED control message only if status level has changed
        """
        if not self.socket_fd:
            return
            
        status_level = self._get_status_level(status_text)
        
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
        """Handle battery status updates"""
        self.send_led_status_if_changed(status)
        
    @pyqtSlot(str)
    def handle_plan_status(self, status: str):
        """Handle plan execution status updates"""
        self.send_led_status_if_changed(status)
        
    def add_status_mapping(self, status_text: str, level: StatusLevel):
        """
        Add or update status level mapping
        Allows dynamic addition of new status mappings
        """
        self.status_level_map[status_text.lower().strip()] = level
        
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
            }
        }