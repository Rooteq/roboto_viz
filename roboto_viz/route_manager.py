import json
from typing import Dict, List, Tuple
from pathlib import Path
import os

# Type alias for clarity
Point4D = Tuple[float, float, float, float]

class RouteManager:
    def __init__(self, app_dir_name: str = ".robotroutes"):
        """
        Initialize RouteManager with configuration directory in user's home.
        
        Args:
            app_dir_name (str): Name of the application directory to create in home folder
        """
        # Setup paths
        self.home_dir = Path.home()
        self.config_dir = self.home_dir / app_dir_name
        self.routes_file = self.config_dir / "routes.json"
        
        # Ensure config directory exists
        self._setup_config_directory()

    def _setup_config_directory(self):
        """Create configuration directory if it doesn't exist."""
        try:
            self.config_dir.mkdir(parents=True, exist_ok=True)
            print(f"Using config directory: {self.config_dir}")
        except PermissionError:
            print(f"Error: No permission to create directory: {self.config_dir}")
            raise
        except Exception as e:
            print(f"Error creating config directory: {str(e)}")
            raise

    def load_routes(self) -> Dict[str, List[Point4D]]:
        """
        Load routes from the JSON file in the config directory.
        Creates a new file if it doesn't exist.
        
        Returns:
            Dict[str, List[Point4D]]: Dictionary of routes with 4D points (x,y,z,w)
        """
        try:
            # Try to open and read the file
            if self.routes_file.exists():
                with open(self.routes_file, 'r') as f:
                    data = json.load(f)
                print(f"Successfully loaded routes from '{self.routes_file}'")
            else:
                # Create new file if it doesn't exist
                data = {}
                with open(self.routes_file, 'w') as f:
                    json.dump(data, f, indent=2)
                print(f"Created new routes file: '{self.routes_file}'")
            
            # Convert lists to tuples for coordinates
            routes = {
                name: [tuple(point) if len(point) == 4 else tuple(point + [0.0] * (4 - len(point)))
                      for point in points]
                for name, points in data.items()
            }
            return routes
            
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON format in '{self.routes_file}': {str(e)}")
            return {}
        except PermissionError:
            print(f"Error: No permission to access '{self.routes_file}'")
            return {}
        except Exception as e:
            print(f"Error loading routes: {str(e)}")
            return {}

    def save_routes(self, routes: Dict[str, List[Point4D]]) -> bool:
        """
        Save routes to the JSON file.
        
        Args:
            routes: Dictionary of routes with 4D points to save
            
        Returns:
            bool: True if save was successful, False otherwise
        """
        try:
            # Convert tuples to lists for JSON serialization
            data = {
                name: [list(point) for point in points]
                for name, points in routes.items()
            }
            
            with open(self.routes_file, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"Successfully saved routes to '{self.routes_file}'")
            return True
            
        except Exception as e:
            print(f"Error saving routes: {str(e)}")
            return False

    def add_route(self, name: str, points: List[Point4D]) -> bool:
        """
        Add a new route to the existing routes.
        
        Args:
            name: Name of the new route
            points: List of 4D points (x,y,z,w) for the route
            
        Returns:
            bool: True if route was added successfully, False otherwise
        """
        # Validate points
        if not all(len(point) == 4 for point in points):
            print("Error: All points must have 4 coordinates (x,y,z,w)")
            return False

        # First load existing routes
        routes = self.load_routes()
        
        # Check if route name already exists
        if name in routes:
            print(f"Error: Route '{name}' already exists")
            return False
        
        # Add new route
        routes[name] = points
        
        # Save updated routes
        return self.save_routes(routes)

    def get_file_path(self) -> str:
        """Return the current routes file path."""
        return str(self.routes_file)