import json
from typing import Dict, List, Optional, Union
from pathlib import Path
from enum import Enum
from dataclasses import dataclass, asdict
from roboto_viz.route_manager import BezierRoute


class ActionType(Enum):
    ROUTE = "route"
    DOCK = "dock"
    UNDOCK = "undock"
    WAIT_FOR_SIGNAL = "wait_for_signal"


@dataclass
class PlanAction:
    action_type: ActionType
    name: str
    parameters: Dict = None
    
    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}
    
    def to_dict(self) -> dict:
        return {
            'action_type': self.action_type.value,
            'name': self.name,
            'parameters': self.parameters
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'PlanAction':
        return cls(
            action_type=ActionType(data['action_type']),
            name=data['name'],
            parameters=data.get('parameters', {})
        )


@dataclass
class ExecutionPlan:
    name: str
    description: str = ""
    map_name: str = ""
    actions: List[PlanAction] = None
    current_action_index: int = 0
    is_active: bool = False
    
    def __post_init__(self):
        if self.actions is None:
            self.actions = []
    
    def add_action(self, action: PlanAction):
        self.actions.append(action)
    
    def remove_action(self, index: int):
        if 0 <= index < len(self.actions):
            del self.actions[index]
            if self.current_action_index >= len(self.actions):
                self.current_action_index = max(0, len(self.actions) - 1)
    
    def get_current_action(self) -> Optional[PlanAction]:
        if 0 <= self.current_action_index < len(self.actions):
            return self.actions[self.current_action_index]
        return None
    
    def next_action(self) -> Optional[PlanAction]:
        if self.current_action_index < len(self.actions) - 1:
            self.current_action_index += 1
            return self.get_current_action()
        else:
            # Loop back to start for continuous execution
            self.current_action_index = 0
            return self.get_current_action()
    
    def set_current_action(self, index: int):
        if 0 <= index < len(self.actions):
            self.current_action_index = index
    
    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'description': self.description,
            'map_name': self.map_name,
            'actions': [action.to_dict() for action in self.actions],
            'current_action_index': self.current_action_index,
            'is_active': self.is_active
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'ExecutionPlan':
        actions = [PlanAction.from_dict(action_data) for action_data in data.get('actions', [])]
        return cls(
            name=data['name'],
            description=data.get('description', ''),
            map_name=data.get('map_name', ''),
            actions=actions,
            current_action_index=data.get('current_action_index', 0),
            is_active=data.get('is_active', False)
        )


class PlanManager:
    def __init__(self, app_dir_name: str = ".robotroutes"):
        self.home_dir = Path.home()
        self.config_dir = self.home_dir / app_dir_name
        self.plans_file = self.config_dir / "plans.json"
        self.plans_dir = self.config_dir / "plans"
        
        # Ensure directories exist
        self._setup_config_directory()
        
        # Current active plan
        self.current_plan: Optional[ExecutionPlan] = None
        
        # All available plans
        self.plans: Dict[str, ExecutionPlan] = {}
        
        # Load existing plans
        self.load_plans()
    
    def _setup_config_directory(self):
        try:
            self.config_dir.mkdir(parents=True, exist_ok=True)
            self.plans_dir.mkdir(parents=True, exist_ok=True)
            print(f"Using plans directory: {self.plans_dir}")
        except PermissionError:
            print(f"Error: No permission to create directory: {self.config_dir}")
            raise
        except Exception as e:
            print(f"Error creating plans directory: {str(e)}")
            raise
    
    def load_plans(self) -> Dict[str, ExecutionPlan]:
        try:
            if self.plans_file.exists():
                with open(self.plans_file, 'r') as f:
                    data = json.load(f)
                print(f"Successfully loaded plans from '{self.plans_file}'")
            else:
                data = {}
                with open(self.plans_file, 'w') as f:
                    json.dump(data, f, indent=2)
                print(f"Created new plans file: '{self.plans_file}'")
            
            # Convert to ExecutionPlan objects
            self.plans = {}
            for name, plan_data in data.items():
                self.plans[name] = ExecutionPlan.from_dict(plan_data)
            
            return self.plans
            
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON format in '{self.plans_file}': {str(e)}")
            return {}
        except Exception as e:
            print(f"Error loading plans: {str(e)}")
            return {}
    
    def save_plans(self) -> bool:
        try:
            plans_data = {name: plan.to_dict() for name, plan in self.plans.items()}
            
            with open(self.plans_file, 'w') as f:
                json.dump(plans_data, f, indent=2)
            print(f"Successfully saved plans to '{self.plans_file}'")
            return True
            
        except Exception as e:
            print(f"Error saving plans: {str(e)}")
            return False
    
    def add_plan(self, plan: ExecutionPlan) -> bool:
        if plan.name in self.plans:
            print(f"Error: Plan '{plan.name}' already exists")
            return False
        
        self.plans[plan.name] = plan
        return self.save_plans()
    
    def remove_plan(self, plan_name: str) -> bool:
        if plan_name not in self.plans:
            print(f"Error: Plan '{plan_name}' does not exist")
            return False
        
        del self.plans[plan_name]
        return self.save_plans()
    
    def update_plan(self, plan: ExecutionPlan) -> bool:
        self.plans[plan.name] = plan
        return self.save_plans()
    
    def get_plan(self, plan_name: str) -> Optional[ExecutionPlan]:
        return self.plans.get(plan_name)
    
    def get_plan_names(self) -> List[str]:
        return list(self.plans.keys())
    
    def set_current_plan(self, plan_name: str) -> bool:
        if plan_name not in self.plans:
            print(f"Error: Plan '{plan_name}' does not exist")
            return False
        
        # Deactivate previous plan
        if self.current_plan:
            self.current_plan.is_active = False
        
        # Set new current plan
        self.current_plan = self.plans[plan_name]
        self.current_plan.is_active = True
        
        # Save the updated state
        return self.save_plans()
    
    def get_current_plan(self) -> Optional[ExecutionPlan]:
        return self.current_plan
    
    def create_route_action(self, route_name: str) -> PlanAction:
        return PlanAction(
            action_type=ActionType.ROUTE,
            name=route_name,
            parameters={'route_name': route_name}
        )
    
    def create_dock_action(self) -> PlanAction:
        return PlanAction(
            action_type=ActionType.DOCK,
            name="Dock Robot",
            parameters={}
        )
    
    def create_undock_action(self) -> PlanAction:
        return PlanAction(
            action_type=ActionType.UNDOCK,
            name="Undock Robot",
            parameters={}
        )
    
    def create_wait_signal_action(self, signal_name: str = "default") -> PlanAction:
        return PlanAction(
            action_type=ActionType.WAIT_FOR_SIGNAL,
            name=f"Wait for {signal_name}",
            parameters={'signal_name': signal_name}
        )
    
    # Note: create_stop_wait_action method removed