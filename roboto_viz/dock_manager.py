import json
from typing import Dict, Optional
from pathlib import Path
import os

class Dock:
    """
    Represents a single dock with position and orientation.
    """
    def __init__(self, x: float, y: float, orientation: float = 0.0):
        self.x = x
        self.y = y
        self.orientation = orientation  # in radians
        
    def to_dict(self) -> dict:
        """Convert dock to dictionary for JSON serialization"""
        return {
            'x': self.x,
            'y': self.y,
            'orientation': self.orientation
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Dock':
        """Create dock from dictionary"""
        return cls(
            x=data['x'],
            y=data['y'],
            orientation=data.get('orientation', 0.0)
        )

class DockManager:
    """
    Manages dock locations with persistence to JSON files.
    Docks are stored per map in ~/.robotroutes/docks.json
    """
    
    def __init__(self):
        self.base_dir = Path.home() / '.robotroutes'
        self.docks_file = self.base_dir / 'docks.json'
        self.current_map: Optional[str] = None
        
        # Ensure directory exists
        self.base_dir.mkdir(exist_ok=True)
        
        # Create empty docks file if it doesn't exist
        if not self.docks_file.exists():
            self.save_docks({})
    
    def load_docks(self) -> Dict[str, Dock]:
        """
        Load docks for the current map.
        
        Returns:
            Dict[str, Dock]: Dictionary mapping dock names to Dock objects
        """
        if not self.current_map:
            return {}
        
        try:
            with open(self.docks_file, 'r') as f:
                all_docks = json.load(f)
            
            map_docks = all_docks.get(self.current_map, {})
            
            # Convert dictionaries back to Dock objects
            docks = {}
            for name, dock_data in map_docks.items():
                docks[name] = Dock.from_dict(dock_data)
            
            return docks
            
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error loading docks: {e}")
            return {}
    
    def save_docks(self, docks: Dict[str, Dock]) -> bool:
        """
        Save docks for the current map.
        
        Args:
            docks: Dictionary mapping dock names to Dock objects
            
        Returns:
            bool: True if saved successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False
        
        try:
            # Load existing data
            try:
                with open(self.docks_file, 'r') as f:
                    all_docks = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                all_docks = {}
            
            # Convert Dock objects to dictionaries
            dock_data = {}
            for name, dock in docks.items():
                dock_data[name] = dock.to_dict()
            
            # Update data for current map
            all_docks[self.current_map] = dock_data
            
            # Save to file
            with open(self.docks_file, 'w') as f:
                json.dump(all_docks, f, indent=2)
            
            return True
            
        except Exception as e:
            print(f"Error saving docks: {e}")
            return False
    
    def add_dock(self, name: str, dock: Dock) -> bool:
        """
        Add a new dock to the existing docks for the current map.
        
        Args:
            name: Name of the new dock
            dock: Dock object to add
            
        Returns:
            bool: True if dock was added successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False

        # First load existing docks
        docks = self.load_docks()
        
        # Check if dock name already exists
        if name in docks:
            print(f"Error: Dock '{name}' already exists for map '{self.current_map}'")
            return False
        
        # Add new dock
        docks[name] = dock
        
        # Save updated docks
        return self.save_docks(docks)

    def update_dock(self, name: str, dock: Dock) -> bool:
        """
        Update an existing dock or add a new one if it doesn't exist.
        
        Args:
            name: Name of the dock to update
            dock: Dock object to save
            
        Returns:
            bool: True if dock was updated successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False

        # Load existing docks
        docks = self.load_docks()
        
        # Update or add the dock (overwrites existing)
        docks[name] = dock
        
        # Save updated docks
        return self.save_docks(docks)

    def remove_dock(self, name: str) -> bool:
        """
        Remove a dock from the current map.
        
        Args:
            name: Name of the dock to remove
            
        Returns:
            bool: True if dock was removed successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False

        # Load existing docks
        docks = self.load_docks()
        
        if name not in docks:
            print(f"Error: Dock '{name}' not found for map '{self.current_map}'")
            return False
        
        # Remove dock
        del docks[name]
        
        # Save updated docks
        return self.save_docks(docks)

    def get_file_path(self) -> str:
        """Return the current docks file path."""
        return str(self.docks_file)

    def set_current_map(self, map_name: str):
        """Set the current map for dock operations."""
        self.current_map = map_name
        print(f"DockManager: Current map set to '{map_name}'")