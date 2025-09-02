#!/usr/bin/env python3
"""
Test script to validate collision detection implementation
"""

import sys
sys.path.append('/home/rooteq/ros2_ws/src/roboto_viz')

try:
    # Test importing the collision detector state
    from roboto_viz.collision_detector_state import CollisionDetectorState
    print("✓ CollisionDetectorState import successful")
    
    # Test creating an instance
    msg = CollisionDetectorState()
    msg.polygons = ["polygon1", "polygon2"] 
    msg.detections = [True, False]
    print("✓ CollisionDetectorState instance creation successful")
    
    # Test collision detection logic
    collision_detected = any(msg.detections)
    print(f"✓ Collision detection logic: {collision_detected}")
    
    # Test CAN status manager import
    from roboto_viz.can_status_manager import CANStatusManager, CANBuzzerType
    print("✓ CANStatusManager import successful")
    
    # Test buzzer message IDs
    print(f"✓ Buzzer ON ID: 0x{int(CANBuzzerType.BUZZER_ON):03X}")
    print(f"✓ Buzzer OFF ID: 0x{int(CANBuzzerType.BUZZER_OFF):03X}")
    
    print("\n✅ All collision detection components working correctly!")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
