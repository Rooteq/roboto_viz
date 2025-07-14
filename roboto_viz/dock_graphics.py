from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem)
from PyQt5.QtGui import QPen, QColor
from PyQt5.QtCore import Qt, QPointF, QObject
from typing import Dict, List

class DockGraphics(QGraphicsItemGroup):
    """
    Visual representation of a dock as a red X marker.
    """
    def __init__(self, x: float, y: float, dock_name: str, parent=None):
        super().__init__(parent)
        
        self.dock_name = dock_name
        self.size = 4  # Size of the X marker (made 2x smaller)
        
        # Create the X using two diagonal lines
        pen = QPen(QColor(255, 0, 0), 3)  # Red, thick line
        pen.setCapStyle(Qt.RoundCap)
        
        # First diagonal line (top-left to bottom-right)
        line1 = QGraphicsLineItem(-self.size, -self.size, self.size, self.size)
        line1.setPen(pen)
        
        # Second diagonal line (top-right to bottom-left)
        line2 = QGraphicsLineItem(self.size, -self.size, -self.size, self.size)
        line2.setPen(pen)
        
        # Add lines to the group
        self.addToGroup(line1)
        self.addToGroup(line2)
        
        # Position the entire group
        self.setPos(x, y)
        
        # Set flags and properties
        self.setFlag(QGraphicsItem.ItemIsMovable, False)  # Disabled by default
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        
        # Track editing state
        self.editing_mode = False
        
        # Enable mouse events
        self.setAcceptedMouseButtons(Qt.LeftButton | Qt.RightButton)
        self.setAcceptHoverEvents(True)
        
        self.setZValue(20)  # Higher than routes and nodes
        
    def itemChange(self, change, value):
        """Handle position changes"""
        if change == QGraphicsItem.ItemPositionHasChanged and self.editing_mode:
            # Only notify about position changes when in editing mode
            if hasattr(self, 'dock_graphics_manager') and self.dock_graphics_manager:
                scene_pos = self.mapToScene(0, 0)
                self.dock_graphics_manager.dock_position_changed(self.dock_name, scene_pos.x(), scene_pos.y())
        return super().itemChange(change, value)
    
    def mousePressEvent(self, event):
        """Handle mouse press for dragging"""
        if event.button() == Qt.LeftButton:
            print(f"DEBUG: *** START DRAGGING dock {self.dock_name} ***")
        elif event.button() == Qt.RightButton:
            print(f"DEBUG: Right-click on dock {self.dock_name} - preparing for deletion")
            return
        super().mousePressEvent(event)
    
    def mouseReleaseEvent(self, event):
        """Handle mouse release for deletion"""
        if event.button() == Qt.RightButton:
            print(f"DEBUG: Right-click release on dock {self.dock_name} - deleting dock")
            if hasattr(self, 'dock_graphics_manager') and self.dock_graphics_manager:
                self.dock_graphics_manager.delete_dock(self.dock_name)
            return
        super().mouseReleaseEvent(event)
    
    def mouseDoubleClickEvent(self, event):
        """Handle double-click for deletion"""
        if event.button() == Qt.LeftButton:
            print(f"DEBUG: Double-click on dock {self.dock_name} - deleting dock")
            if hasattr(self, 'dock_graphics_manager') and self.dock_graphics_manager:
                self.dock_graphics_manager.delete_dock(self.dock_name)
            return
        super().mouseDoubleClickEvent(event)
    
    def set_editing_mode(self, enabled: bool):
        """Enable or disable editing mode for this dock"""
        self.editing_mode = enabled
        self.setFlag(QGraphicsItem.ItemIsMovable, enabled)
        print(f"DEBUG: Dock {self.dock_name} editing mode: {enabled}")
    
    def get_current_position(self) -> tuple:
        """Get current position in scene coordinates"""
        scene_pos = self.mapToScene(0, 0)
        return scene_pos.x(), scene_pos.y()

class DockGraphicsManager(QObject):
    """
    Manages the visual representation of all docks on the map.
    """
    def __init__(self, dock_manager, map_origin, pixmap_height, scene, parent=None):
        super().__init__(parent)
        
        self.scene_ref = scene
        self.dock_manager = dock_manager
        self.map_origin = map_origin
        self.pixmap_height = pixmap_height
        
        # Visual components
        self.dock_items: Dict[str, DockGraphics] = {}
        
        self.update_graphics()
        
    def clear_graphics(self):
        """Remove all dock graphics items from the scene"""
        scene = self.scene_ref
        if scene:
            for item in self.dock_items.values():
                if item.scene():
                    scene.removeItem(item)
        
        self.dock_items.clear()
        
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
        """Update all visual elements based on current dock data"""
        print(f"DEBUG: DockGraphicsManager.update_graphics called")
        
        # Clear existing items
        self.clear_graphics()
        
        # Get the scene to add items to
        scene = self.scene_ref
        if not scene:
            print(f"DEBUG: No scene available, returning")
            return
            
        # Get current docks
        docks = self.dock_manager.load_docks()
        print(f"DEBUG: Found {len(docks)} docks to create (hidden by default)")
            
        # Create dock items but don't add them to scene yet (they start hidden)
        for dock_name, dock in docks.items():
            map_x, map_y = self.world_to_map_coords(dock.x, dock.y)
            print(f"DEBUG: Dock {dock_name}: world=({dock.x}, {dock.y}) -> map=({map_x}, {map_y})")
            
            dock_item = DockGraphics(map_x, map_y, dock_name, None)
            dock_item.dock_graphics_manager = self  # Store reference
            
            self.dock_items[dock_name] = dock_item
            # Don't add to scene yet - docks are hidden by default
            print(f"DEBUG: Created dock {dock_name} (hidden)")
    
    def show_dock(self, dock_name: str):
        """Show a specific dock on the map"""
        if dock_name in self.dock_items and self.scene_ref:
            dock_item = self.dock_items[dock_name]
            if not dock_item.scene():  # Only add if not already in scene
                self.scene_ref.addItem(dock_item)
                print(f"DEBUG: Showing dock {dock_name}")
    
    def hide_dock(self, dock_name: str):
        """Hide a specific dock from the map"""
        if dock_name in self.dock_items:
            dock_item = self.dock_items[dock_name]
            if dock_item.scene():  # Only remove if in scene
                dock_item.scene().removeItem(dock_item)
                print(f"DEBUG: Hiding dock {dock_name}")
    
    def hide_all_docks(self):
        """Hide all docks from the map"""
        for dock_name in self.dock_items:
            self.hide_dock(dock_name)
        print(f"DEBUG: Hidden all docks")
    
    def show_all_docks(self):
        """Show all docks on the map"""
        for dock_name in self.dock_items:
            self.show_dock(dock_name)
        print(f"DEBUG: Showing all docks")
    
    def dock_position_changed(self, dock_name: str, map_x: float, map_y: float):
        """Handle dock position change during editing (doesn't save automatically)"""
        world_x, world_y = self.map_to_world_coords(map_x, map_y)
        print(f"DEBUG: dock_position_changed {dock_name}, map_coords=({map_x}, {map_y}), world_coords=({world_x}, {world_y})")
        # Position change is tracked but not saved until user confirms
    
    def save_dock_position(self, dock_name: str) -> bool:
        """Save the current position of a dock"""
        if dock_name in self.dock_items:
            dock_item = self.dock_items[dock_name]
            map_x, map_y = dock_item.get_current_position()
            world_x, world_y = self.map_to_world_coords(map_x, map_y)
            
            # Update dock position in dock manager
            docks = self.dock_manager.load_docks()
            if dock_name in docks:
                dock = docks[dock_name]
                dock.x = world_x
                dock.y = world_y
                self.dock_manager.save_docks(docks)
                print(f"DEBUG: Saved dock {dock_name} position: ({world_x}, {world_y})")
                return True
        return False
        
    def delete_dock(self, dock_name: str):
        """Delete a dock"""
        # First remove from graphics immediately to ensure it disappears
        if dock_name in self.dock_items:
            dock_item = self.dock_items[dock_name]
            if dock_item.scene():
                dock_item.scene().removeItem(dock_item)
            del self.dock_items[dock_name]
            print(f"DEBUG: Removed dock graphics for {dock_name}")
        
        # Then remove from dock manager and refresh
        if self.dock_manager.remove_dock(dock_name):
            self.update_graphics()
            print(f"DEBUG: Deleted dock {dock_name} from manager")
        else:
            print(f"DEBUG: Failed to delete dock {dock_name} from manager")
        
    def add_dock_at_position(self, map_x: float, map_y: float, dock_name: str):
        """Add a new dock at the specified map position"""
        from roboto_viz.dock_manager import Dock
        world_x, world_y = self.map_to_world_coords(map_x, map_y)
        new_dock = Dock(world_x, world_y)
        if self.dock_manager.add_dock(dock_name, new_dock):
            self.update_graphics()
            return True
        return False
    
    def create_temporary_dock(self, map_x: float, map_y: float, dock_name: str):
        """Create a temporary dock for editing (not saved to dock manager yet)"""
        # Create dock graphics item that can be edited
        dock_item = DockGraphics(map_x, map_y, dock_name, None)
        dock_item.dock_graphics_manager = self
        dock_item.set_editing_mode(True)  # Enable editing immediately
        
        # Store in dock items but don't save to dock manager yet
        self.dock_items[dock_name] = dock_item
        
        # Add to scene for display and editing
        if self.scene_ref:
            self.scene_ref.addItem(dock_item)
            print(f"DEBUG: Created temporary dock {dock_name} at ({map_x}, {map_y}) for editing")
            return True
        return False
    
    def save_temporary_dock(self, dock_name: str) -> bool:
        """Save a temporary dock to the dock manager"""
        if dock_name in self.dock_items:
            dock_item = self.dock_items[dock_name]
            map_x, map_y = dock_item.get_current_position()
            world_x, world_y = self.map_to_world_coords(map_x, map_y)
            
            # Save to dock manager
            from roboto_viz.dock_manager import Dock
            new_dock = Dock(world_x, world_y)
            if self.dock_manager.add_dock(dock_name, new_dock):
                # Disable editing mode for the saved dock
                dock_item.set_editing_mode(False)
                print(f"DEBUG: Saved temporary dock {dock_name} to dock manager")
                return True
        return False
    
    def cancel_temporary_dock(self, dock_name: str):
        """Cancel a temporary dock creation"""
        if dock_name in self.dock_items:
            dock_item = self.dock_items[dock_name]
            if dock_item.scene():
                dock_item.scene().removeItem(dock_item)
            del self.dock_items[dock_name]
            print(f"DEBUG: Cancelled temporary dock {dock_name}")
    
    def set_dock_editing_mode(self, dock_name: str, enabled: bool):
        """Enable or disable editing mode for a specific dock"""
        if dock_name in self.dock_items:
            self.dock_items[dock_name].set_editing_mode(enabled)
    
    def disable_all_dock_editing(self):
        """Disable editing mode for all docks"""
        for dock_item in self.dock_items.values():
            dock_item.set_editing_mode(False)
    
    def revert_dock_position(self, dock_name: str):
        """Revert dock to its saved position"""
        if dock_name in self.dock_items:
            docks = self.dock_manager.load_docks()
            if dock_name in docks:
                dock = docks[dock_name]
                map_x, map_y = self.world_to_map_coords(dock.x, dock.y)
                self.dock_items[dock_name].setPos(map_x, map_y)
                print(f"DEBUG: Reverted dock {dock_name} to saved position")
    
    def force_remove_dock_graphics(self, dock_name: str):
        """Force remove dock graphics even if not in dock_items"""
        scene = self.scene_ref
        if scene:
            # Search through all scene items to find and remove stuck dock graphics
            items_to_remove = []
            for item in scene.items():
                if hasattr(item, 'dock_name') and item.dock_name == dock_name:
                    items_to_remove.append(item)
            
            for item in items_to_remove:
                scene.removeItem(item)
                print(f"DEBUG: Force removed stuck dock graphics for {dock_name}")
        
        # Also remove from dock_items if present
        if dock_name in self.dock_items:
            del self.dock_items[dock_name]