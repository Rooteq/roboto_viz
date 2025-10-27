# roboto_viz
Visualization gui app for roboto_diffbot project

IMPORTANT TODO: 
- add checking for found dock location - abort if varies much;

Status CAN messages:
  - CAN ID 0x201: Turn on Green LED (robot OK)
  - CAN ID 0x202: Turn on Orange LED (robot warning)
  - CAN ID 0x203: Turn on Red LED (robot error)
  0x204 Turn on Buzzer
  0x205 Turn off Buzzer

Change the OK can message to be sent only once

Great! Actually, don't use the parameter get at the start to load default zone. Just let it be somewhere in the code (tell me where so I can change that). Make sure that if the robot starts at some zone, it loads its polygon points.

Collision logic:

How It Works:
Collision Subscriber (gui_manager.py line 590-602):
def collision_callback(self, msg):
    if msg.detections:
        collision_detected = any(msg.detections)
        if self.collision_detection_callback:
            self.collision_detection_callback(collision_detected)
Callback Setup (gui_manager.py line 1287):
self.node.collision_detection_callback = self.emit_collision_detection
Signal Emission (gui_manager.py line 1326-1332):
def emit_collision_detection(self, collision_detected: bool):
    self.collision_detected.emit(collision_detected)
    # Signal is handled by plan_active_view.handle_collision_detection
Signal Connection (gui_state_machine.py line 537-540):
self.connect_and_store_connections(
    self.gui.gui_manager.collision_detected,
    self.gui.main_view.plan_active_view.handle_collision_detection
)
Display Handler (plan_views.py line 582-595):
def handle_collision_detection(self, collision_detected: bool):
    # Only show obstacle status when navigating
    if self.is_navigating:
        if collision_detected:
            self.obstacle_detected = True
            self.set_current_status("Wykryto przeszkodę!")  # Orange background
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.set_current_status("Navigating")  # Restore normal status
Navigation State Tracking (plan_views.py line 562-576):
def update_robot_status(self, status: str):
    # Track navigation state
    self.is_navigating = any(word in status_lower for word in ['nav', 'navigating', 'executing'])
    
    # Don't override obstacle status during navigation
    if not self.obstacle_detected or not self.is_navigating:
        self.set_current_status(status)
Orange Color Application (plan_views.py line 419-423):
# Matches "Wykryto przeszkodę!" and any variation with "wykryto przeszkodę"
elif ("wykryto przeszkodę" in status.lower() or ...):
    bg_color = "#fdebd0"  # Light orange
    text_color = "#f39c12"  # Orange text
Result: When the robot detects an obstacle DURING NAVIGATION:
CAN message sent (buzzer ON + orange LED)
Robot status displays: "Wykryto przeszkodę!"
Status cell turns orange
When obstacle clears: buzzer OFF, green LED, status returns to "Navigating"
Complete Flow Diagram
Robot Navigating
    ↓
Collision Detected (collision_monitor_manager)
    ↓
collision_callback() in ManagerNode
    ↓
emit_collision_detection(True) in GuiManager
    ↓
┌─────────────────────────────────┬──────────────────────────────────┐
│                                 │                                  │
collision_detected signal         (CAN signal also emitted)         
    ↓                                 ↓                               
handle_collision_detection()      handle_collision_detection()      
in plan_active_view              in can_manager                    
    ↓                                 ↓                               
Check: is_navigating?             Send: BUZZER_ON + ORANGE_LED      
    ↓ YES                                                             
set_current_status("Wykryto przeszkodę!")                           
    ↓                                                                 
Apply orange background (#fdebd0)                                   
Apply orange text (#f39c12)                                         