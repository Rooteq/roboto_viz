import json
from typing import Dict, List, Tuple, Optional
from pathlib import Path
import os
import subprocess
import math

# Type alias for clarity
Point4D = Tuple[float, float, float, float]
MapResult = Tuple[bool, str]  # Success flag and message

class RouteNode:
    """
    Represents a single node in a route with position and optional control points for Bezier curves.
    """
    def __init__(self, x: float, y: float, orientation: float = 0.0, 
                 control_in: Optional[Tuple[float, float]] = None,
                 control_out: Optional[Tuple[float, float]] = None):
        self.x = x
        self.y = y
        self.orientation = orientation  # in radians
        self.control_in = control_in    # Control point for incoming curve
        self.control_out = control_out  # Control point for outgoing curve
        
    def to_dict(self) -> dict:
        """Convert node to dictionary for JSON serialization"""
        return {
            'x': self.x,
            'y': self.y,
            'orientation': self.orientation,
            'control_in': self.control_in,
            'control_out': self.control_out
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'RouteNode':
        """Create node from dictionary"""
        return cls(
            x=data['x'],
            y=data['y'],
            orientation=data.get('orientation', 0.0),
            control_in=data.get('control_in'),
            control_out=data.get('control_out')
        )

class BezierRoute:
    """
    Represents a route made of nodes connected with Bezier curves.
    """
    def __init__(self, nodes: List[RouteNode] = None):
        self.nodes = nodes or []
    
    def add_node(self, node: RouteNode):
        """Add a node to the route"""
        self.nodes.append(node)
        self._auto_generate_control_points_selective()
    
    def remove_node(self, index: int):
        """Remove node at index"""
        if 0 <= index < len(self.nodes):
            del self.nodes[index]
            self._auto_generate_control_points_selective()
    
    def move_node(self, index: int, new_x: float, new_y: float):
        """Move node to new position"""
        if 0 <= index < len(self.nodes):
            self.nodes[index].x = new_x
            self.nodes[index].y = new_y
            # Don't auto-regenerate control points when moving nodes manually
            # Control points are moved along with the node in the graphics layer
    
    def _auto_generate_control_points(self):
        """Automatically generate control points for smooth curves"""
        if len(self.nodes) < 2:
            return
            
        for i in range(len(self.nodes)):
            node = self.nodes[i]
            
            # Calculate control points based on neighboring nodes
            if i == 0:
                # First node - only outgoing control
                if len(self.nodes) > 1:
                    next_node = self.nodes[1]
                    dx = next_node.x - node.x
                    dy = next_node.y - node.y
                    length = math.sqrt(dx*dx + dy*dy) * 0.3  # 30% of distance
                    angle = math.atan2(dy, dx)
                    node.control_out = (
                        node.x + length * math.cos(angle),
                        node.y + length * math.sin(angle)
                    )
                    node.control_in = None
            elif i == len(self.nodes) - 1:
                # Last node - only incoming control
                prev_node = self.nodes[i-1]
                dx = node.x - prev_node.x
                dy = node.y - prev_node.y
                length = math.sqrt(dx*dx + dy*dy) * 0.3
                angle = math.atan2(dy, dx)
                node.control_in = (
                    node.x - length * math.cos(angle),
                    node.y - length * math.sin(angle)
                )
                node.control_out = None
            else:
                # Middle node - both controls
                prev_node = self.nodes[i-1]
                next_node = self.nodes[i+1]
                
                # Calculate smooth tangent
                dx_in = node.x - prev_node.x
                dy_in = node.y - prev_node.y
                dx_out = next_node.x - node.x
                dy_out = next_node.y - node.y
                
                # Average direction for smooth curve
                avg_angle = math.atan2(dy_in + dy_out, dx_in + dx_out)
                
                in_length = math.sqrt(dx_in*dx_in + dy_in*dy_in) * 0.3
                out_length = math.sqrt(dx_out*dx_out + dy_out*dy_out) * 0.3
                
                node.control_in = (
                    node.x - in_length * math.cos(avg_angle),
                    node.y - in_length * math.sin(avg_angle)
                )
                node.control_out = (
                    node.x + out_length * math.cos(avg_angle),
                    node.y + out_length * math.sin(avg_angle)
                )
    
    def _auto_generate_control_points_selective(self):
        """
        Automatically generate control points only for nodes that don't have them,
        preserving manually positioned control points.
        """
        if len(self.nodes) < 2:
            return
            
        for i in range(len(self.nodes)):
            node = self.nodes[i]
            
            # Only generate control points if they don't already exist
            # This preserves manually positioned control points
            
            # Calculate control points based on neighboring nodes
            if i == 0:
                # First node - only outgoing control
                if len(self.nodes) > 1 and node.control_out is None:
                    next_node = self.nodes[1]
                    dx = next_node.x - node.x
                    dy = next_node.y - node.y
                    length = math.sqrt(dx*dx + dy*dy) * 0.3  # 30% of distance
                    angle = math.atan2(dy, dx)
                    node.control_out = (
                        node.x + length * math.cos(angle),
                        node.y + length * math.sin(angle)
                    )
                # Always clear control_in for first node
                if node.control_in is not None:
                    node.control_in = None
            elif i == len(self.nodes) - 1:
                # Last node - only incoming control
                if node.control_in is None:
                    prev_node = self.nodes[i-1]
                    dx = node.x - prev_node.x
                    dy = node.y - prev_node.y
                    length = math.sqrt(dx*dx + dy*dy) * 0.3
                    angle = math.atan2(dy, dx)
                    node.control_in = (
                        node.x - length * math.cos(angle),
                        node.y - length * math.sin(angle)
                    )
                # Always clear control_out for last node
                if node.control_out is not None:
                    node.control_out = None
            else:
                # Middle node - both controls (only if they don't exist)
                if node.control_in is None or node.control_out is None:
                    prev_node = self.nodes[i-1]
                    next_node = self.nodes[i+1]
                    
                    # Calculate smooth tangent
                    dx_in = node.x - prev_node.x
                    dy_in = node.y - prev_node.y
                    dx_out = next_node.x - node.x
                    dy_out = next_node.y - node.y
                    
                    # Average direction for smooth curve
                    avg_angle = math.atan2(dy_in + dy_out, dx_in + dx_out)
                    
                    in_length = math.sqrt(dx_in*dx_in + dy_in*dy_in) * 0.3
                    out_length = math.sqrt(dx_out*dx_out + dy_out*dy_out) * 0.3
                    
                    if node.control_in is None:
                        node.control_in = (
                            node.x - in_length * math.cos(avg_angle),
                            node.y - in_length * math.sin(avg_angle)
                        )
                    if node.control_out is None:
                        node.control_out = (
                            node.x + out_length * math.cos(avg_angle),
                            node.y + out_length * math.sin(avg_angle)
                        )
    
    def generate_waypoints(self, points_per_segment: int = 20) -> List[Point4D]:
        """
        Generate dense waypoints along the Bezier curves for navigation.
        Returns list of (x, y, z, orientation) tuples.
        """
        if len(self.nodes) < 2:
            # Single node or empty route
            return [(node.x, node.y, 0.0, node.orientation) for node in self.nodes]
        
        waypoints = []
        
        for i in range(len(self.nodes) - 1):
            start_node = self.nodes[i]
            end_node = self.nodes[i + 1]
            
            # Generate points along Bezier curve
            for j in range(points_per_segment):
                t = j / points_per_segment
                
                # Cubic Bezier calculation
                p0 = (start_node.x, start_node.y)
                p1 = start_node.control_out or (start_node.x, start_node.y)
                p2 = end_node.control_in or (end_node.x, end_node.y)
                p3 = (end_node.x, end_node.y)
                
                x, y = self._cubic_bezier_point(t, p0, p1, p2, p3)
                
                # Calculate orientation as tangent to curve
                if j < points_per_segment - 1:
                    t_next = (j + 1) / points_per_segment
                    x_next, y_next = self._cubic_bezier_point(t_next, p0, p1, p2, p3)
                    orientation = math.atan2(y_next - y, x_next - x)
                else:
                    orientation = end_node.orientation
                
                waypoints.append((x, y, 0.0, orientation))
        
        # Add final node
        final_node = self.nodes[-1]
        waypoints.append((final_node.x, final_node.y, 0.0, final_node.orientation))
        
        return waypoints
    
    def _cubic_bezier_point(self, t: float, p0: Tuple[float, float], 
                           p1: Tuple[float, float], p2: Tuple[float, float], 
                           p3: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate point on cubic Bezier curve at parameter t (0-1)"""
        u = 1 - t
        tt = t * t
        uu = u * u
        uuu = uu * u
        ttt = tt * t
        
        x = uuu * p0[0] + 3 * uu * t * p1[0] + 3 * u * tt * p2[0] + ttt * p3[0]
        y = uuu * p0[1] + 3 * uu * t * p1[1] + 3 * u * tt * p2[1] + ttt * p3[1]
        
        return (x, y)
    
    def to_dict(self) -> dict:
        """Convert route to dictionary for JSON serialization"""
        return {
            'nodes': [node.to_dict() for node in self.nodes]
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'BezierRoute':
        """Create route from dictionary"""
        nodes = [RouteNode.from_dict(node_data) for node_data in data.get('nodes', [])]
        return cls(nodes)

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

    def load_routes(self) -> Dict[str, BezierRoute]:
        """
        Load routes from the JSON file for the current map.
        Creates a new file if it doesn't exist.
        
        Returns:
            Dict[str, BezierRoute]: Dictionary of BezierRoute objects
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
            
            # Convert to BezierRoute objects
            routes = {}
            for name, route_data in map_routes.items():
                # Check if it's new node-based format or old point-based format
                if isinstance(route_data, dict) and 'nodes' in route_data:
                    # New format - BezierRoute
                    routes[name] = BezierRoute.from_dict(route_data)
                elif isinstance(route_data, list):
                    # Old format - convert points to simple nodes
                    bezier_route = BezierRoute()
                    for point in route_data:
                        if len(point) >= 4:
                            node = RouteNode(point[0], point[1], point[3])
                            bezier_route.add_node(node)
                    routes[name] = bezier_route
            
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

    def save_routes(self, routes: Dict[str, BezierRoute]) -> bool:
        """
        Save routes to the JSON file for the current map.
        
        Args:
            routes: Dictionary of BezierRoute objects to save
            
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
            
            # Convert BezierRoute objects to dictionaries for JSON serialization
            if self.current_map:
                all_routes[self.current_map] = {
                    name: route.to_dict()
                    for name, route in routes.items()
                }
            
            with open(self.routes_file, 'w') as f:
                json.dump(all_routes, f, indent=2)
            print(f"Successfully saved routes for map '{self.current_map}' to '{self.routes_file}'")
            return True
            
        except Exception as e:
            print(f"Error saving routes: {str(e)}")
            return False

    def add_route(self, name: str, route: BezierRoute) -> bool:
        """
        Add a new route to the existing routes for the current map.
        
        Args:
            name: Name of the new route
            route: BezierRoute object to add
            
        Returns:
            bool: True if route was added successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False

        # First load existing routes
        routes = self.load_routes()
        
        # Check if route name already exists
        if name in routes:
            print(f"Error: Route '{name}' already exists for map '{self.current_map}'")
            return False
        
        # Add new route
        routes[name] = route
        
        # Save updated routes
        return self.save_routes(routes)

    def update_route(self, name: str, route: BezierRoute) -> bool:
        """
        Update an existing route or add a new one if it doesn't exist.
        
        Args:
            name: Name of the route to update
            route: BezierRoute object to save
            
        Returns:
            bool: True if route was updated successfully, False otherwise
        """
        if not self.current_map:
            print("Error: No map selected")
            return False

        # Load existing routes
        routes = self.load_routes()
        
        # Update or add the route (overwrites existing)
        routes[name] = route
        
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