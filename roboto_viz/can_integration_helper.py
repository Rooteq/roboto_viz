#!/usr/bin/env python3
"""
Helper functions for integrating CAN status messages with the existing GUI architecture.
This module provides easy-to-use functions that can be added to the state machine
to forward all status updates to CAN messages.
"""

def connect_status_forwarding_to_can(gui_manager, active_view=None, plan_active_view=None, plan_executor=None):
    """
    Connect status forwarding to CAN for all major status types.
    
    This function should be called during state setup (e.g., in ConfiguringState or ActiveState)
    to ensure all status updates are forwarded to CAN messages.
    
    Args:
        gui_manager: The GuiManager instance with CAN capabilities
        active_view: ActiveView instance (optional)
        plan_active_view: PlanActiveView instance (optional) 
        plan_executor: PlanExecutor instance (optional)
    """
    
    if not gui_manager or not gui_manager.can_manager:
        print("CAN manager not available - status forwarding disabled")
        return
    
    # 1. Forward all robot status updates via wrapper lambdas
    def forward_robot_status(status):
        gui_manager.send_robot_status_to_can(status)
    
    def forward_battery_status(status):
        gui_manager.send_battery_status_to_can(status)
        
    def forward_plan_status(status):
        gui_manager.send_plan_status_to_can(status)
    
    # 2. Connect robot status from manual operations
    gui_manager.manualStatus.connect(forward_robot_status)
    
    # 3. Connect robot status from docking operations  
    gui_manager.dockingStatus.connect(forward_robot_status)
    
    # 4. Connect robot status from navigation operations
    if hasattr(gui_manager.navigator, 'navStatus'):
        gui_manager.navigator.navStatus.connect(forward_robot_status)
    
    # 5. Connect plan execution status updates
    if plan_executor and hasattr(plan_executor, 'status_update'):
        plan_executor.status_update.connect(forward_plan_status)
    
    # 6. Connect navigation preparation signals for buzzer control
    if plan_executor and hasattr(plan_executor, 'navigation_preparation_started'):
        plan_executor.navigation_preparation_started.connect(gui_manager.can_manager.send_navigation_preparation_message)

    if plan_executor and hasattr(plan_executor, 'navigation_preparation_stopped'):
        plan_executor.navigation_preparation_stopped.connect(gui_manager.can_manager.stop_navigation_preparation)

    print("CAN status forwarding connections established")


def create_can_status_forwarder_signals(gui_manager):
    """
    Create PyQt signal connections that forward status updates to CAN.
    This is an alternative approach that creates reusable lambda functions.
    
    Returns a dictionary of forwarder functions that can be connected to status signals.
    """
    
    if not gui_manager or not gui_manager.can_manager:
        return {}
    
    forwarders = {
        'robot_status': lambda status: gui_manager.send_robot_status_to_can(status),
        'battery_status': lambda status: gui_manager.send_battery_status_to_can(status), 
        'plan_status': lambda status: gui_manager.send_plan_status_to_can(status),
        'navigation_status': lambda status: gui_manager.send_robot_status_to_can(status),  # Nav status goes to robot status
        'manual_status': lambda status: gui_manager.send_robot_status_to_can(status),     # Manual status goes to robot status
        'docking_status': lambda status: gui_manager.send_robot_status_to_can(status),    # Docking status goes to robot status
    }
    
    return forwarders


def add_can_forwarding_to_state_connections(state_instance, gui_manager):
    """
    Add CAN forwarding connections to an existing state's connection setup.
    This can be called from within a state's setup_connections method.
    
    Args:
        state_instance: The state instance that has connect_and_store_connections method
        gui_manager: GuiManager instance with CAN support
    """
    
    if not gui_manager or not gui_manager.can_manager:
        return
    
    # Get forwarder functions
    forwarders = create_can_status_forwarder_signals(gui_manager)
    
    if not forwarders:
        return
    
    # Connect all the major status signals to CAN forwarders
    # These use the state's existing connection tracking system
    
    # Robot status from manual operations
    state_instance.connect_and_store_connections(
        gui_manager.manualStatus,
        forwarders['manual_status']
    )
    
    # Robot status from docking operations
    state_instance.connect_and_store_connections(
        gui_manager.dockingStatus, 
        forwarders['docking_status']
    )
    
    # Robot status from navigation operations
    if hasattr(gui_manager.navigator, 'navStatus'):
        state_instance.connect_and_store_connections(
            gui_manager.navigator.navStatus,
            forwarders['navigation_status']  
        )
    
    # Plan status from plan executor (if we can access it through the state)
    if hasattr(state_instance, 'gui') and hasattr(state_instance.gui, 'main_view'):
        if hasattr(state_instance.gui.main_view, 'plan_executor'):
            plan_executor = state_instance.gui.main_view.plan_executor
            if hasattr(plan_executor, 'status_update'):
                state_instance.connect_and_store_connections(
                    plan_executor.status_update,
                    forwarders['plan_status']
                )
            # Connect navigation preparation signals for buzzer control
            if hasattr(plan_executor, 'navigation_preparation_started') and hasattr(state_instance.gui, 'gui_manager') and state_instance.gui.gui_manager.can_manager:
                state_instance.connect_and_store_connections(
                    plan_executor.navigation_preparation_started,
                    state_instance.gui.gui_manager.can_manager.send_navigation_preparation_message
                )
            if hasattr(plan_executor, 'navigation_preparation_stopped') and hasattr(state_instance.gui, 'gui_manager') and state_instance.gui.gui_manager.can_manager:
                state_instance.connect_and_store_connections(
                    plan_executor.navigation_preparation_stopped,
                    state_instance.gui.gui_manager.can_manager.stop_navigation_preparation
                )
    
    print("CAN forwarding added to state connections")


# Example usage patterns:
"""
# Usage Pattern 1: In gui_state_machine.py ConfiguringState or ActiveState
def setup_connections(self):
    # ... existing connections ...
    
    # Add CAN forwarding
    from roboto_viz.can_integration_helper import add_can_forwarding_to_state_connections
    add_can_forwarding_to_state_connections(self, self.gui.gui_manager)


# Usage Pattern 2: Direct connection in state setup
def setup_connections(self):
    # ... existing connections ...
    
    from roboto_viz.can_integration_helper import connect_status_forwarding_to_can
    connect_status_forwarding_to_can(
        self.gui.gui_manager,
        self.gui.main_view.active_view,
        self.gui.main_view.plan_active_view,
        self.gui.main_view.plan_executor
    )


# Usage Pattern 3: Manual connection with forwarders
def setup_connections(self):
    # ... existing connections ...
    
    from roboto_viz.can_integration_helper import create_can_status_forwarder_signals
    forwarders = create_can_status_forwarder_signals(self.gui.gui_manager)
    
    if forwarders:
        # Connect specific signals
        self.connect_and_store_connections(
            self.gui.gui_manager.manualStatus,
            forwarders['manual_status']
        )
"""