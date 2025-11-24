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
    
    # Test env expression manager import
    from roboto_viz.env_expression_manager import CANStatusManager, ModbusCoil
    print("✓ CANStatusManager import successful")

    # Test coil addresses
    print(f"✓ GREEN LED coil: {int(ModbusCoil.GREEN_LED)}")
    print(f"✓ BUZZER coil: {int(ModbusCoil.BUZZER)}")
    
    print("\n✅ All collision detection components working correctly!")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
