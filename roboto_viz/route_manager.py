import json
from typing import Dict, List, Tuple, Optional
from pathlib import Path
import os
import subprocess

# Type alias for clarity
Point4D = Tuple[float, float, float, float]
MapResult = Tuple[bool, str]  # Success flag and message

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
        self.maps_dir = self.config_dir / "maps"
        
        # Current active map
        self.current_map = None
        
        # Ensure config directory exists
        self._setup_config_directory()

    def _setup_config_directory(self):
        """Create configuration directory and maps subdirectory if they don't exist."""
        try:
            self.config_dir.mkdir(parents=True, exist_ok=True)
            self.maps_dir.mkdir(parents=True, exist_ok=True)
            print(f"Using config directory: {self.config_dir}")
            print(f"Using maps directory: {self.maps_dir}")
        except PermissionError:
            print(f"Error: No permission to create directory: {self.config_dir}")
            raise
        except Exception as e:
            print(f"Error creating config directory: {str(e)}")
            raise

    def load_routes(self) -> Dict[str, List[Point4D]]:
        """
        Load routes from the JSON file for the current map.
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
            
            # Get routes for current map
            map_routes = data.get(self.current_map, {}) if self.current_map else {}
            
            # Convert lists to tuples for coordinates
            routes = {
                name: [tuple(point) if len(point) == 4 else tuple(point + [0.0] * (4 - len(point)))
                      for point in points]
                for name, points in map_routes.items()
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
        Save routes to the JSON file for the current map.
        
        Args:
            routes: Dictionary of routes with 4D points to save
            
        Returns:
            bool: True if save was successful, False otherwise
        """
        try:
            # First load all existing routes
            if self.routes_file.exists():
                with open(self.routes_file, 'r') as f:
                    all_routes = json.load(f)
            else:
                all_routes = {}
            
            # Convert tuples to lists for JSON serialization
            if self.current_map:
                all_routes[self.current_map] = {
                    name: [list(point) for point in points]
                    for name, points in routes.items()
                }
            
            with open(self.routes_file, 'w') as f:
                json.dump(all_routes, f, indent=2)
            print(f"Successfully saved routes for map '{self.current_map}' to '{self.routes_file}'")
            return True
            
        except Exception as e:
            print(f"Error saving routes: {str(e)}")
            return False

    def add_route(self, name: str, points: List[Point4D]) -> bool:
        """
        Add a new route to the existing routes for the current map.
        
        Args:
            name: Name of the new route
            points: List of 4D points (x,y,z,w) for the route
            
        Returns:
            bool: True if route was added successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False

        # Validate points
        if not all(len(point) == 4 for point in points):
            print("Error: All points must have 4 coordinates (x,y,z,w)")
            return False

        # First load existing routes
        routes = self.load_routes()
        
        # Check if route name already exists
        if name in routes:
            print(f"Error: Route '{name}' already exists for map '{self.current_map}'")
            return False
        
        # Add new route
        routes[name] = points
        
        # Save updated routes
        return self.save_routes(routes)

    def get_file_path(self) -> str:
        """Return the current routes file path."""
        return str(self.routes_file)

    def set_current_map(self, map_name: str):
        """
        Set the current active map and load its routes.
        
        Args:
            map_name: Name of the map to set as current
        """
        self.current_map = map_name
        print(f"Set current map to: {map_name}")

    def load_map(self, map_name: str) -> MapResult:
        """
        Save the current robot's map using nav2_map_server.
        
        Args:
            map_name: Name to save the map as
            
        Returns:
            Tuple[bool, str]: (Success flag, Error message if failed or empty string if successful)
        """
        try:
            # Create the full path for the map
            map_path = self.maps_dir / map_name
            
            # Construct the command
            cmd = [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", str(map_path),
                "--ros-args",
                "-p", "map_subscribe_transient_local:=true"
            ]
            
            # Execute the command
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True
            )
            
            # Check if both .pgm and .yaml files were created
            pgm_file = map_path.with_suffix('.pgm')
            yaml_file = map_path.with_suffix('.yaml')
            
            if pgm_file.exists() and yaml_file.exists():
                print(f"Successfully saved map '{map_name}' to {self.maps_dir}")
                return True, ""
            else:
                return False, "Map files were not created properly"
                
        except subprocess.CalledProcessError as e:
            error_msg = f"Map saving failed: {e.stderr}"
            print(error_msg)
            return False, error_msg
        except Exception as e:
            error_msg = f"Unexpected error: {str(e)}"
            print(error_msg)
            return False, error_msg

    def get_map_names(self) -> List[str]:
        """
        Get a list of all available maps in the maps directory.
        Only returns names of maps that have both .yaml and .pgm files.
        
        Returns:
            List[str]: List of valid map names without file extensions
        """
        try:
            # Get all .yaml files in the maps directory
            yaml_files = list(self.maps_dir.glob("*.yaml"))
            
            # Extract just the map names (without path and extension)
            map_names = [file.stem for file in yaml_files]
            
            # Verify each map name has both .yaml and .pgm files
            valid_maps = []
            for map_name in map_names:
                yaml_path = self.maps_dir / f"{map_name}.yaml"
                pgm_path = self.maps_dir / f"{map_name}.pgm"
                
                if yaml_path.exists() and pgm_path.exists():
                    valid_maps.append(map_name)
                    
            valid_maps.sort()  # Sort for consistent ordering
            print(f"Found {len(valid_maps)} valid maps")
            return valid_maps
            
        except Exception as e:
            print(f"Error getting map names: {str(e)}")
            return []
    
    
    def load_map_onto_robot(self, map_name: str) -> MapResult:
        """
        Load a map onto the robot using ros2 service call command.
        
        Args:
            map_name: Name of the map file (without extension)
            
        Returns:
            Tuple[bool, str]: (Success flag, Error message if failed or empty string if successful)
        """
        try:
            # Construct full path to map YAML file
            map_path = self.maps_dir / f"{map_name}.yaml"
            
            if not map_path.exists():
                error_msg = f"Map file not found: {map_path}"
                print(error_msg)
                return False, error_msg

            # Construct the ros2 service call command
            cmd = [
                "ros2", "service", "call",
                "/map_server/load_map",
                "nav2_msgs/srv/LoadMap",
                f"{{map_url: '{str(map_path)}'}}"
            ]
            
            # Execute the command
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True  # This will raise CalledProcessError if the command fails
            )
            
            # Check if the command was successful
            if "success: True" in result.stdout:
                print(f"Successfully loaded map '{map_name}' onto robot")
                return True, ""
            else:
                error_msg = f"Failed to load map: {result.stdout}"
                print(error_msg)
                return False, error_msg
                
        except subprocess.CalledProcessError as e:
            error_msg = f"Error loading map onto robot: {e.stderr}"
            print(error_msg)
            return False, error_msg
        except Exception as e:
            error_msg = f"Unexpected error: {str(e)}"
            print(error_msg)
            return False, error_msg