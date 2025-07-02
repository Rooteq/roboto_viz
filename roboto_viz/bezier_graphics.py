from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsEllipseItem, QGraphicsPathItem, 
                             QGraphicsLineItem, QGraphicsItemGroup, QGraphicsTextItem)
from PyQt5.QtGui import QPainterPath, QPen, QBrush, QColor, QFont
from PyQt5.QtCore import Qt, QPointF, QRectF, QObject
import math

class BezierNode(QGraphicsEllipseItem):
    """
    Visual representation of a route node that can be moved and edited.
    """
    def __init__(self, x: float, y: float, index: int, parent=None):
        radius = 4
        super().__init__(x - radius, y - radius, radius * 2, radius * 2, parent)
        
        self.index = index
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)  # Disable selection to prevent group dragging
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        
        # Visual styling
        self.setBrush(QBrush(QColor(50, 150, 250)))  # Blue
        self.setPen(QPen(QColor(30, 100, 200), 2))
        
        # Enable mouse events for this item
        self.setAcceptedMouseButtons(Qt.LeftButton | Qt.RightButton)
        self.setAcceptHoverEvents(True)
        
        # Add number label
        self.text_item = QGraphicsTextItem(str(index), self)
        font = QFont()
        font.setPointSize(6)
        font.setBold(True)
        self.text_item.setFont(font)
        self.text_item.setDefaultTextColor(Qt.white)
        
        # Center the text relative to the ellipse center
        text_rect = self.text_item.boundingRect()
        self.text_item.setPos(
            radius - text_rect.width()/2, 
            radius - text_rect.height()/2
        )
        
        # IMPORTANT: Make sure text doesn't interfere with mouse events
        self.text_item.setFlag(QGraphicsItem.ItemIsMovable, False)
        self.text_item.setFlag(QGraphicsItem.ItemIsSelectable, False)
        self.text_item.setFlag(QGraphicsItem.ItemStacksBehindParent, True)  # Add this
        self.text_item.setAcceptedMouseButtons(Qt.NoButton)
        self.text_item.setAcceptHoverEvents(False)
        
        self.setZValue(10)  # Higher than curves
        
    def itemChange(self, change, value):
        """Handle position changes"""
        if change == QGraphicsItem.ItemPositionHasChanged:
            # Notify route graphics about position change - use the center position
            if hasattr(self, 'route_graphics') and self.route_graphics:
                # Use the actual scene coordinates instead of the value parameter
                # The value parameter is relative to parent, but we need scene coordinates
                scene_pos = self.mapToScene(self.rect().center())
                
                print(f"DEBUG: Node {self.index} moved to scene position: {scene_pos.x()}, {scene_pos.y()}")
                self.route_graphics.node_moved(self.index, scene_pos.x(), scene_pos.y())
        return super().itemChange(change, value)
    
    def mousePressEvent(self, event):
        """Handle mouse press for dragging"""
        print(f"DEBUG: BezierNode.mousePressEvent called for node {self.index}")
        if event.button() == Qt.LeftButton:
            print(f"DEBUG: *** START DRAGGING node {self.index} at {event.pos()} ***")
            # Don't call setSelected() to avoid selection bounding box
        super().mousePressEvent(event)

class ControlHandle(QGraphicsEllipseItem):
    """
    Visual representation of Bezier control points.
    """
    def __init__(self, x: float, y: float, node_index: int, is_out: bool, parent=None):
        radius = 2
        super().__init__(x - radius, y - radius, radius * 2, radius * 2, parent)
        
        self.node_index = node_index
        self.is_out = is_out  # True for out control, False for in control
        
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        
        # Visual styling - smaller and different color
        self.setBrush(QBrush(QColor(150, 150, 150)))  # Gray
        self.setPen(QPen(QColor(100, 100, 100), 1))
        
        # Enable mouse events for this item
        self.setAcceptedMouseButtons(Qt.LeftButton | Qt.RightButton)
        self.setAcceptHoverEvents(True)
        
        self.setZValue(9)  # Below nodes but above curves
        
    def itemChange(self, change, value):
        """Handle position changes"""
        if change == QGraphicsItem.ItemPositionHasChanged:
            # Notify route graphics about control point movement - use the center position
            if hasattr(self, 'route_graphics') and self.route_graphics:
                # Use the actual scene coordinates instead of the value parameter
                scene_pos = self.mapToScene(self.rect().center())
                
                self.route_graphics.control_moved(
                    self.node_index, self.is_out, scene_pos.x(), scene_pos.y())
        return super().itemChange(change, value)
    
    def mousePressEvent(self, event):
        """Handle mouse press for dragging"""
        print(f"DEBUG: ControlHandle.mousePressEvent called for handle {self.node_index}")
        if event.button() == Qt.LeftButton:
            print(f"DEBUG: *** START DRAGGING control handle {self.node_index} ***")
            # Don't call setSelected() to avoid selection bounding box
        super().mousePressEvent(event)

class BezierCurve(QGraphicsPathItem):
    """
    Visual representation of a Bezier curve between two nodes.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Visual styling
        pen = QPen(QColor(0, 200, 0), 3)  # Green, thick line
        pen.setCapStyle(Qt.RoundCap)
        self.setPen(pen)
        
        self.setZValue(1)  # Below nodes and controls
        
    def update_curve(self, p0: QPointF, p1: QPointF, p2: QPointF, p3: QPointF):
        """
        Update the curve with new control points.
        p0: start point, p1: start control, p2: end control, p3: end point
        """
        path = QPainterPath()
        path.moveTo(p0)
        path.cubicTo(p1, p2, p3)
        self.setPath(path)

class ControlLine(QGraphicsLineItem):
    """
    Visual line connecting nodes to their control handles.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Visual styling - dashed line
        pen = QPen(QColor(150, 150, 150), 1)
        pen.setStyle(Qt.DashLine)
        self.setPen(pen)
        
        self.setZValue(2)

class BezierRouteGraphics(QObject):
    """
    Complete visual representation of a Bezier route with nodes, curves, and controls.
    """
    def __init__(self, bezier_route, map_origin, pixmap_height, scene, parent=None):
        super().__init__(parent)
        
        self.scene_ref = scene
        
        self.bezier_route = bezier_route
        self.map_origin = map_origin
        self.pixmap_height = pixmap_height
        
        # Visual components
        self.node_items = []
        self.curve_items = []
        self.control_handles = []
        self.control_lines = []
        
        self.update_graphics()
        
    def clear_graphics(self):
        """Remove all graphics items from the scene"""
        scene = self.scene_ref
        if scene:
            for item in self.node_items + self.curve_items + self.control_handles + self.control_lines:
                if item.scene():
                    scene.removeItem(item)
        
        self.node_items.clear()
        self.curve_items.clear()
        self.control_handles.clear()
        self.control_lines.clear()
        
    def world_to_map_coords(self, world_x: float, world_y: float) -> tuple:
        """Convert world coordinates to map pixel coordinates"""
        map_x = (world_x - self.map_origin[0]) * 20
        map_y = self.pixmap_height - ((world_y - self.map_origin[1]) * 20)
        return map_x, map_y
    
    def map_to_world_coords(self, map_x: float, map_y: float) -> tuple:
        """Convert map pixel coordinates to world coordinates"""
        world_x = (map_x / 20) + self.map_origin[0]
        world_y = ((self.pixmap_height - map_y) / 20) + self.map_origin[1]
        return world_x, world_y
        
    def update_graphics(self):
        """Update all visual elements based on current route data"""
        print(f"DEBUG: update_graphics called with {len(self.bezier_route.nodes)} nodes")
        
        # Clear existing items
        for item in self.node_items + self.curve_items + self.control_handles + self.control_lines:
            if item.scene():
                item.scene().removeItem(item)
        
        self.node_items.clear()
        self.curve_items.clear()
        self.control_handles.clear()
        self.control_lines.clear()
        
        if not self.bezier_route.nodes:
            print(f"DEBUG: No nodes in route, returning")
            return
            
        # Get the scene to add items to
        scene = self.scene_ref
        if not scene:
            print(f"DEBUG: No scene available, returning")
            return
        
        print(f"DEBUG: Scene available, creating graphics for {len(self.bezier_route.nodes)} nodes")
            
        # Create node items - add directly to scene
        for i, node in enumerate(self.bezier_route.nodes):
            map_x, map_y = self.world_to_map_coords(node.x, node.y)
            print(f"DEBUG: Node {i}: world=({node.x}, {node.y}) -> map=({map_x}, {map_y})")
            node_item = BezierNode(map_x, map_y, i, None)  # No parent - will be added to scene
            node_item.route_graphics = self  # Store reference to this graphics object
            self.node_items.append(node_item)
            scene.addItem(node_item)
            print(f"DEBUG: Added node {i} to scene")
            
            # Create control handles if they exist
            if node.control_in:
                cx, cy = self.world_to_map_coords(node.control_in[0], node.control_in[1])
                control_in = ControlHandle(cx, cy, i, False, None)
                control_in.route_graphics = self
                self.control_handles.append(control_in)
                scene.addItem(control_in)
                
                # Control line
                line = ControlLine()
                line.setLine(map_x, map_y, cx, cy)
                self.control_lines.append(line)
                scene.addItem(line)
                
            if node.control_out:
                cx, cy = self.world_to_map_coords(node.control_out[0], node.control_out[1])
                control_out = ControlHandle(cx, cy, i, True, None)
                control_out.route_graphics = self
                self.control_handles.append(control_out)
                scene.addItem(control_out)
                
                # Control line
                line = ControlLine()
                line.setLine(map_x, map_y, cx, cy)
                self.control_lines.append(line)
                scene.addItem(line)
        
        # Create curve items
        for i in range(len(self.bezier_route.nodes) - 1):
            curve = BezierCurve()
            self.curve_items.append(curve)
            scene.addItem(curve)
            self.update_curve(i)
    
    def update_curve(self, curve_index: int):
        """Update a specific curve between two nodes"""
        if curve_index >= len(self.curve_items):
            return
            
        start_node = self.bezier_route.nodes[curve_index]
        end_node = self.bezier_route.nodes[curve_index + 1]
        
        # Convert to map coordinates
        p0_x, p0_y = self.world_to_map_coords(start_node.x, start_node.y)
        p3_x, p3_y = self.world_to_map_coords(end_node.x, end_node.y)
        
        p0 = QPointF(p0_x, p0_y)
        p3 = QPointF(p3_x, p3_y)
        
        # Control points
        if start_node.control_out:
            p1_x, p1_y = self.world_to_map_coords(start_node.control_out[0], start_node.control_out[1])
            p1 = QPointF(p1_x, p1_y)
        else:
            p1 = p0
            
        if end_node.control_in:
            p2_x, p2_y = self.world_to_map_coords(end_node.control_in[0], end_node.control_in[1])
            p2 = QPointF(p2_x, p2_y)
        else:
            p2 = p3
            
        self.curve_items[curve_index].update_curve(p0, p1, p2, p3)
    
    def node_moved(self, index: int, map_x: float, map_y: float):
        """Handle node movement"""
        if index >= len(self.bezier_route.nodes):
            return
            
        world_x, world_y = self.map_to_world_coords(map_x, map_y)
        print(f"DEBUG: node_moved index={index}, map_coords=({map_x}, {map_y}), world_coords=({world_x}, {world_y})")
        
        # Calculate how much the node moved
        node = self.bezier_route.nodes[index]
        dx = world_x - node.x
        dy = world_y - node.y
        
        # Update node position
        node.x = world_x
        node.y = world_y
        
        # Move control points by the same amount to maintain relative position
        if node.control_in:
            node.control_in = (node.control_in[0] + dx, node.control_in[1] + dy)
        if node.control_out:
            node.control_out = (node.control_out[0] + dx, node.control_out[1] + dy)
        
        # Update visual control handles to match the new positions
        self.move_visual_control_handles(index, dx, dy)
        
        # Update control lines for this node
        self.update_control_lines_for_node(index)
        
        # Update curves
        if index > 0:
            self.update_curve(index - 1)
        if index < len(self.bezier_route.nodes) - 1:
            self.update_curve(index)
        
    def control_moved(self, node_index: int, is_out: bool, map_x: float, map_y: float):
        """Handle control point movement"""
        world_x, world_y = self.map_to_world_coords(map_x, map_y)
        
        node = self.bezier_route.nodes[node_index]
        if is_out:
            node.control_out = (world_x, world_y)
        else:
            node.control_in = (world_x, world_y)
            
        # Update affected curves
        if is_out and node_index < len(self.bezier_route.nodes) - 1:
            self.update_curve(node_index)
        if not is_out and node_index > 0:
            self.update_curve(node_index - 1)
            
        # Update only the control line for this specific control point
        self.update_control_line(node_index, is_out, map_x, map_y)
        
    def delete_node(self, index: int):
        """Delete a node"""
        self.bezier_route.remove_node(index)
        self.update_graphics()
        
    def add_node_at_position(self, map_x: float, map_y: float):
        """Add a new node at the specified map position"""
        world_x, world_y = self.map_to_world_coords(map_x, map_y)
        new_node = RouteNode(world_x, world_y)
        self.bezier_route.add_node(new_node)
        self.update_graphics()
        
    def update_node_connections(self, node_index: int):
        """Update curves and control lines connected to a specific node without recreating all graphics"""
        if node_index >= len(self.bezier_route.nodes):
            return
            
        node = self.bezier_route.nodes[node_index]
        map_x, map_y = self.world_to_map_coords(node.x, node.y)
        
        # Update control lines connected to this node
        control_line_index = 0
        for i, bezier_node in enumerate(self.bezier_route.nodes):
            if i == node_index:
                # Update control lines for this node
                if bezier_node.control_in and control_line_index < len(self.control_lines):
                    cx, cy = self.world_to_map_coords(bezier_node.control_in[0], bezier_node.control_in[1])
                    self.control_lines[control_line_index].setLine(map_x, map_y, cx, cy)
                    control_line_index += 1
                    
                if bezier_node.control_out and control_line_index < len(self.control_lines):
                    cx, cy = self.world_to_map_coords(bezier_node.control_out[0], bezier_node.control_out[1])
                    self.control_lines[control_line_index].setLine(map_x, map_y, cx, cy)
                    control_line_index += 1
            else:
                # Skip control lines for other nodes
                if bezier_node.control_in:
                    control_line_index += 1
                if bezier_node.control_out:
                    control_line_index += 1
        
        # Update curves connected to this node
        # Update curve from previous node to this node
        if node_index > 0:
            self.update_curve(node_index - 1)
            
        # Update curve from this node to next node
        if node_index < len(self.bezier_route.nodes) - 1:
            self.update_curve(node_index)

    def update_control_line(self, node_index: int, is_out: bool, control_x: float, control_y: float):
        """Update a specific control line without recreating all graphics"""
        if node_index >= len(self.bezier_route.nodes):
            return
            
        node = self.bezier_route.nodes[node_index]
        node_map_x, node_map_y = self.world_to_map_coords(node.x, node.y)
        
        # Find the correct control line index
        control_line_index = 0
        for i, bezier_node in enumerate(self.bezier_route.nodes):
            if i == node_index:
                if is_out:
                    # If we're looking for the out control, skip the in control if it exists
                    if bezier_node.control_in:
                        control_line_index += 1
                    # Now we're at the out control line
                    if control_line_index < len(self.control_lines):
                        self.control_lines[control_line_index].setLine(node_map_x, node_map_y, control_x, control_y)
                else:
                    # We're looking for the in control, which comes first
                    if control_line_index < len(self.control_lines):
                        self.control_lines[control_line_index].setLine(node_map_x, node_map_y, control_x, control_y)
                break
            else:
                # Count control lines for previous nodes
                if bezier_node.control_in:
                    control_line_index += 1
                if bezier_node.control_out:
                    control_line_index += 1
                    
    def update_control_handle_positions(self, node_index: int):
        """Update the positions of control handles for a specific node"""
        if node_index >= len(self.bezier_route.nodes):
            return
        
        # Find control handles for this node
        control_handle_index = 0
        for i, bezier_node in enumerate(self.bezier_route.nodes):
            if i == node_index:
                # Update control handles for this node
                if bezier_node.control_in and control_handle_index < len(self.control_handles):
                    cx, cy = self.world_to_map_coords(bezier_node.control_in[0], bezier_node.control_in[1])
                    # Move the control handle to new position (accounting for radius offset)
                    self.control_handles[control_handle_index].setPos(cx - 2, cy - 2)
                    control_handle_index += 1
                    
                if bezier_node.control_out and control_handle_index < len(self.control_handles):
                    cx, cy = self.world_to_map_coords(bezier_node.control_out[0], bezier_node.control_out[1])
                    # Move the control handle to new position (accounting for radius offset)
                    self.control_handles[control_handle_index].setPos(cx - 2, cy - 2)
                    control_handle_index += 1
            else:
                # Skip control handles for other nodes
                if bezier_node.control_in:
                    control_handle_index += 1
                if bezier_node.control_out:
                    control_handle_index += 1
                    
    def move_visual_control_handles(self, node_index: int, dx_world: float, dy_world: float):
        """Move the visual control handles by the specified world coordinate offset"""
        # Convert world coordinate offset to map coordinate offset
        dx_map = dx_world * 20  # Same scale factor used in coordinate conversion
        dy_map = -dy_world * 20  # Y is inverted in map coordinates
        
        # Find control handles for this node and move them
        control_handle_index = 0
        for i, bezier_node in enumerate(self.bezier_route.nodes):
            if i == node_index:
                # Move control handles for this node
                if bezier_node.control_in and control_handle_index < len(self.control_handles):
                    current_pos = self.control_handles[control_handle_index].pos()
                    self.control_handles[control_handle_index].setPos(current_pos.x() + dx_map, current_pos.y() + dy_map)
                    control_handle_index += 1
                    
                if bezier_node.control_out and control_handle_index < len(self.control_handles):
                    current_pos = self.control_handles[control_handle_index].pos()
                    self.control_handles[control_handle_index].setPos(current_pos.x() + dx_map, current_pos.y() + dy_map)
                    control_handle_index += 1
                break
            else:
                # Skip control handles for other nodes
                if bezier_node.control_in:
                    control_handle_index += 1
                if bezier_node.control_out:
                    control_handle_index += 1
                    
    def update_control_lines_for_node(self, node_index: int):
        """Update control lines for a specific node after it has moved"""
        if node_index >= len(self.bezier_route.nodes):
            return
            
        node = self.bezier_route.nodes[node_index]
        node_map_x, node_map_y = self.world_to_map_coords(node.x, node.y)
        
        # Find control lines for this node and update them
        control_line_index = 0
        for i, bezier_node in enumerate(self.bezier_route.nodes):
            if i == node_index:
                # Update control lines for this node
                if bezier_node.control_in and control_line_index < len(self.control_lines):
                    cx, cy = self.world_to_map_coords(bezier_node.control_in[0], bezier_node.control_in[1])
                    self.control_lines[control_line_index].setLine(node_map_x, node_map_y, cx, cy)
                    control_line_index += 1
                    
                if bezier_node.control_out and control_line_index < len(self.control_lines):
                    cx, cy = self.world_to_map_coords(bezier_node.control_out[0], bezier_node.control_out[1])
                    self.control_lines[control_line_index].setLine(node_map_x, node_map_y, cx, cy)
                    control_line_index += 1
                break
            else:
                # Skip control lines for other nodes
                if bezier_node.control_in:
                    control_line_index += 1
                if bezier_node.control_out:
                    control_line_index += 1

# Import the RouteNode here to avoid circular imports
from roboto_viz.route_manager import RouteNode
