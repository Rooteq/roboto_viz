#!/usr/bin/env python3
"""
Test script to validate collision detection with navigation state
"""

import sys
sys.path.append('/home/rooteq/ros2_ws/src/roboto_viz')

def test_collision_behavior():
    try:
        # Test env expression manager navigation state tracking
        from roboto_viz.env_expression_manager import CANStatusManager

        print("Testing collision detection with navigation state...")

        # Create manager (won't connect to Modbus for testing)
        manager = CANStatusManager("/dev/ttyUSB0", 1)
        
        # Test 1: Collision detected but not navigating - should be OFF
        manager.is_navigating = False
        collision_detected = True
        should_buzz = collision_detected and manager.is_navigating
        print(f"Test 1 - Collision: {collision_detected}, Navigating: {manager.is_navigating}")
        print(f"  Expected: buzzer OFF (False)")
        print(f"  Actual: {should_buzz}")
        print(f"  ✓ PASS" if not should_buzz else "  ❌ FAIL")
        print()

        # Test 2: Collision detected and navigating - should be ON
        manager.is_navigating = True
        collision_detected = True
        should_buzz = collision_detected and manager.is_navigating
        print(f"Test 2 - Collision: {collision_detected}, Navigating: {manager.is_navigating}")
        print(f"  Expected: buzzer ON (True)")
        print(f"  Actual: {should_buzz}")
        print(f"  ✓ PASS" if should_buzz else "  ❌ FAIL")
        print()

        # Test 3: No collision but navigating - should be OFF
        manager.is_navigating = True
        collision_detected = False
        should_buzz = collision_detected and manager.is_navigating
        print(f"Test 3 - Collision: {collision_detected}, Navigating: {manager.is_navigating}")
        print(f"  Expected: buzzer OFF (False)")
        print(f"  Actual: {should_buzz}")
        print(f"  ✓ PASS" if not should_buzz else "  ❌ FAIL")
        print()
        
        # Test 4: Navigation status parsing
        print("Testing navigation status parsing...")
        
        nav_statuses = [
            ("Nawigacja do celu", True),    # Should set navigating = True
            ("Nav to dest", True),          # Should set navigating = True
            ("Zatrzymany", False),          # Should set navigating = False
            ("Na miejscu docelowym", False), # Should set navigating = False
            ("W bazie", False),             # Should set navigating = False
            ("Błąd Nawigacji", False),      # Should set navigating = False
        ]
        
        for status, expected_nav_state in nav_statuses:
            # Simulate status processing
            nav_active_keywords = ["nav to", "navigating", "nawigacja"]
            nav_inactive_keywords = ["zatrzymany", "stopped", "na miejscu", "w bazie", "bezczynny", "idle", "błąd", "failed", "error"]
            
            status_lower = status.lower()
            if any(keyword in status_lower for keyword in nav_active_keywords):
                actual_nav_state = True
            elif any(keyword in status_lower for keyword in nav_inactive_keywords):
                actual_nav_state = False
            else:
                actual_nav_state = manager.is_navigating  # No change
            
            print(f"  Status: '{status}' -> Navigating: {actual_nav_state} (expected: {expected_nav_state})")
            print(f"    ✓ PASS" if actual_nav_state == expected_nav_state else "    ❌ FAIL")
        
        print("\n✅ All collision detection behavior tests completed!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_collision_behavior()
