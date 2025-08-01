from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QSlider, QComboBox, QGroupBox,
                             QMessageBox, QButtonGroup)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QPainter, QPen, QBrush, QColor
from pathlib import Path
import yaml
import shutil
from typing import Optional


class SpeedZoneEditor(QWidget):
    """Widget for editing speed zones on PGM maps"""
    
    speed_zone_updated = pyqtSignal()
    editing_finished = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_map_path: Optional[str] = None
        self.current_yaml_path: Optional[str] = None
        self.original_map_path: Optional[str] = None
        self.speed_mask_pixmap: Optional[QPixmap] = None
        self.drawing_enabled = False
        self.current_tool = 'paint'  # 'paint' or 'erase'
        self.current_speed_ms = 0.3  # Current speed in m/s (default 0.3 m/s)
        self.drawing_box = False
        self.box_start_pos = None
        self.box_end_pos = None
        self.map_name = None
        
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Apply global scaling for large screens (1920x1080)
        self.setStyleSheet("""
            QWidget {
                font-size: 14px;
            }
            QLabel {
                font-size: 14px;
            }
            QPushButton {
                font-size: 16px;
                min-height: 40px;
                padding: 8px 16px;
                font-weight: bold;
                border: 2px solid #2c3e50;
                border-radius: 8px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
            QComboBox {
                font-size: 14px;
                min-height: 35px;
                padding: 5px;
            }
            QGroupBox {
                font-size: 16px;
                font-weight: bold;
                padding-top: 25px;
                margin-top: 10px;
            }
        """)
        
        # Title
        title = QLabel('Speed Zone Editor')
        title.setStyleSheet('font-weight: bold; font-size: 18px;')
        layout.addWidget(title)
        
        # Tool selection
        tools_group = QGroupBox("Tools")
        tools_layout = QVBoxLayout(tools_group)
        
        # Tool buttons
        tool_buttons_layout = QHBoxLayout()
        self.tool_group = QButtonGroup()
        
        self.paint_btn = QPushButton("Paint Zone")
        self.paint_btn.setCheckable(True)
        self.paint_btn.setChecked(True)
        self.tool_group.addButton(self.paint_btn, 0)
        tool_buttons_layout.addWidget(self.paint_btn)
        
        self.erase_btn = QPushButton("Erase Zone")
        self.erase_btn.setCheckable(True)
        self.tool_group.addButton(self.erase_btn, 1)
        tool_buttons_layout.addWidget(self.erase_btn)
        
        tools_layout.addLayout(tool_buttons_layout)
        
        # Speed setting
        speed_layout = QVBoxLayout()
        speed_layout.addWidget(QLabel("Speed Limit (m/s):"))
        
        self.speed_combo = QComboBox()
        self.speed_combo.addItems([
            "0.1 m/s",
            "0.2 m/s", 
            "0.3 m/s",
            "0.4 m/s",
            "0.5 m/s"
        ])
        self.speed_combo.setCurrentIndex(2)  # Default to 0.3 m/s
        speed_layout.addWidget(self.speed_combo)
        
        tools_layout.addLayout(speed_layout)
        
        
        layout.addWidget(tools_group)
        
        # Map operations
        map_ops_group = QGroupBox("Map Operations")
        map_ops_layout = QVBoxLayout(map_ops_group)
        
        # Clear button
        ops_layout = QHBoxLayout()
        
        self.clear_zones_btn = QPushButton("Clear All Zones")
        
        ops_layout.addWidget(self.clear_zones_btn)
        map_ops_layout.addLayout(ops_layout)
        
        # Save/Cancel buttons
        save_layout = QHBoxLayout()
        
        self.save_speed_mask_btn = QPushButton("Save Speed Mask")
        self.save_speed_mask_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
                font-size: 16px;
                min-height: 40px;
                padding: 12px 20px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
        """)
        
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-size: 16px;
                font-weight: bold;
                min-height: 40px;
                padding: 12px 20px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        
        save_layout.addWidget(self.save_speed_mask_btn)
        save_layout.addWidget(self.cancel_btn)
        map_ops_layout.addLayout(save_layout)
        
        layout.addWidget(map_ops_group)
        
        layout.addStretch()
        
        self.setup_connections()
        
    def setup_connections(self):
        self.tool_group.buttonToggled.connect(self.on_tool_changed)
        self.speed_combo.currentIndexChanged.connect(self.on_speed_changed)
        self.clear_zones_btn.clicked.connect(self.clear_all_zones)
        self.save_speed_mask_btn.clicked.connect(self.save_speed_mask)
        self.cancel_btn.clicked.connect(self.cancel_editing)
        
    def on_tool_changed(self, button, checked):
        if checked:
            if button == self.paint_btn:
                self.current_tool = "paint"
            elif button == self.erase_btn:
                self.current_tool = "erase"
    
    def on_speed_changed(self, index):
        # Map combo index to speed in m/s
        speeds = [0.1, 0.2, 0.3, 0.4, 0.5]
        self.current_speed_ms = speeds[index]
        
    def load_map(self, map_name: str):
        """Load a map for speed zone editing"""
        print(f"DEBUG: SpeedZoneEditor.load_map called with: {map_name}")
        maps_dir = Path.home() / ".robotroutes" / "maps"
        
        # Use speed_ prefixed map for editing
        speed_map_path = maps_dir / f"speed_{map_name}.pgm"
        speed_yaml_path = maps_dir / f"speed_{map_name}.yaml"
        original_map_path = maps_dir / f"{map_name}.pgm"
        
        print(f"DEBUG: Looking for speed map: {speed_map_path}")
        print(f"DEBUG: Speed map exists: {speed_map_path.exists()}")
        print(f"DEBUG: Speed yaml exists: {speed_yaml_path.exists()}")
        
        if not speed_map_path.exists() or not speed_yaml_path.exists():
            print(f"DEBUG: Speed map files not found for '{map_name}'")
            QMessageBox.warning(self, "Error", f"Speed map files not found for '{map_name}'!")
            return False
            
        # Store paths
        self.current_map_path = str(speed_map_path)
        self.current_yaml_path = str(speed_yaml_path)
        self.original_map_path = str(original_map_path)
        self.map_name = map_name
        
        # Load the speed_ prefixed map for editing
        self.speed_mask_pixmap = QPixmap(self.current_map_path)
        self.drawing_enabled = True
        
        return True
        
    def get_display_map_path(self):
        """Get the path to the current working map (for display purposes)"""
        return self.current_map_path if self.current_map_path else None
        
    def clear_all_zones(self):
        """Clear all speed zones by loading the original map"""
        reply = QMessageBox.question(self, 'Clear Zones', 
                                   'Are you sure you want to clear all speed zones?',
                                   QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.original_map_path and self.current_map_path:
                try:
                    # Copy original to speed_ prefixed map
                    shutil.copy2(self.original_map_path, self.current_map_path)
                    self.speed_mask_pixmap = QPixmap(self.current_map_path)
                    QMessageBox.information(self, "Success", "All speed zones cleared!")
                    self.speed_zone_updated.emit()
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Failed to clear speed zones: {str(e)}")
            
    def get_gray_value_for_speed(self, speed_ms: float) -> int:
        """Convert speed in m/s to gray value for speed mask
        
        For nav2 speed filter:
        - Higher gray values = higher speed
        - Speed scaling: gray_value = (speed_ms / max_speed) * 255
        """
        max_speed = 0.5  # Maximum speed
        return int((speed_ms / max_speed) * 255)
        
    def start_box_drawing(self, scene_x: float, scene_y: float):
        """Start drawing a box at the given coordinates"""
        print(f"DEBUG: start_box_drawing at ({scene_x}, {scene_y})")
        self.drawing_box = True
        self.box_start_pos = (scene_x, scene_y)
        self.box_end_pos = (scene_x, scene_y)
        
    def update_box_drawing(self, scene_x: float, scene_y: float):
        """Update box end position during dragging"""
        if self.drawing_box:
            self.box_end_pos = (scene_x, scene_y)
            
    def finish_box_drawing(self, scene_x: float, scene_y: float):
        """Finish drawing the box and apply it to the map"""
        print(f"DEBUG: finish_box_drawing at ({scene_x}, {scene_y})")
        print(f"DEBUG: drawing_box={self.drawing_box}, drawing_enabled={self.drawing_enabled}, pixmap={self.speed_mask_pixmap is not None}")
        
        if not self.drawing_box or not self.drawing_enabled or not self.speed_mask_pixmap:
            print(f"DEBUG: Aborting box drawing - conditions not met")
            return
            
        self.box_end_pos = (scene_x, scene_y)
        
        # Get box coordinates
        x1, y1 = self.box_start_pos
        x2, y2 = self.box_end_pos
        
        # Ensure coordinates are in correct order
        min_x, max_x = int(min(x1, x2)), int(max(x1, x2))
        min_y, max_y = int(min(y1, y2)), int(max(y1, y2))
        
        # Check bounds
        min_x = max(0, min_x)
        min_y = max(0, min_y)
        max_x = min(self.speed_mask_pixmap.width() - 1, max_x)
        max_y = min(self.speed_mask_pixmap.height() - 1, max_y)
        
        if min_x >= max_x or min_y >= max_y:
            self.drawing_box = False
            return
            
        # Create painter for the pixmap
        painter = QPainter(self.speed_mask_pixmap)
        
        if self.current_tool == 'paint':
            # Paint speed zone
            gray_value = self.get_gray_value_for_speed(self.current_speed_ms)
            color = QColor(gray_value, gray_value, gray_value)
        else:
            # Erase (paint with white/free space)
            color = QColor(255, 255, 255)  # White for free space
            
        painter.fillRect(min_x, min_y, max_x - min_x, max_y - min_y, color)
        painter.end()
        
        # Save the updated pixmap back to file
        try:
            self.speed_mask_pixmap.save(self.current_map_path)
            print(f"DEBUG: Saved speed zone changes to {self.current_map_path}")
            self.speed_zone_updated.emit()
        except Exception as e:
            print(f"Error saving pixmap: {e}")
            
        self.drawing_box = False
        
    def cancel_box_drawing(self):
        """Cancel current box drawing"""
        self.drawing_box = False
        self.box_start_pos = None
        self.box_end_pos = None
            
    def save_speed_mask(self):
        """Save the current speed zones with proper speed_ prefix and nav2-compatible YAML"""
        if not self.current_map_path or not self.current_yaml_path or not self.map_name:
            QMessageBox.warning(self, "Error", "No map loaded!")
            return
            
        try:
            # Get map directory
            map_path = Path(self.current_map_path)
            maps_dir = map_path.parent
            
            # Use speed_ prefixed naming (maintaining consistency)
            speed_map_pgm = maps_dir / f"speed_{self.map_name}.pgm"
            speed_map_yaml = maps_dir / f"speed_{self.map_name}.yaml"
            
            # Save current working map (already has speed_ prefix)
            # The current_map_path should already be the speed_ prefixed file
            if str(map_path) != str(speed_map_pgm):
                shutil.copy2(self.current_map_path, speed_map_pgm)
            
            # Read the original map's YAML to get base properties
            original_yaml_path = maps_dir / f"{self.map_name}.yaml"
            if original_yaml_path.exists():
                with open(original_yaml_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
            else:
                # Fallback to current yaml if original doesn't exist
                with open(self.current_yaml_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
            
            # Update YAML for nav2 speed restrictions according to specifications
            yaml_data['image'] = f'speed_{self.map_name}.pgm'
            yaml_data['mode'] = 'scale'
            yaml_data['occupied_thresh'] = 1.0
            yaml_data['free_thresh'] = 0.0
            
            # Ensure origin has zero yaw component for Costmap2D compatibility
            if 'origin' in yaml_data and len(yaml_data['origin']) >= 3:
                yaml_data['origin'][2] = 0
            
            # Write speed map YAML with nav2-compatible parameters
            with open(speed_map_yaml, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
                
            QMessageBox.information(self, "Success", 
                                  f"Speed map saved as:\n{speed_map_pgm}\n{speed_map_yaml}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save speed map: {str(e)}")
            
    def cancel_editing(self):
        """Cancel speed zone editing and return to original map"""
        reply = QMessageBox.question(self, 'Cancel Editing', 
                                   'Cancel speed zone editing? Any unsaved changes will be lost.',
                                   QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.cleanup_editing()
            
    def cleanup_editing(self):
        """Clean up editing state"""
        # Reset all editing state
        self.drawing_enabled = False
        self.drawing_box = False
        self.box_start_pos = None
        self.box_end_pos = None
        
        # Clear pixmap reference
        self.speed_mask_pixmap = None
        
        # Note: We keep the speed_ prefixed files (don't delete them)
        # Clear file paths
        self.current_map_path = None
        self.original_map_path = None
        self.current_yaml_path = None
        self.map_name = None
        
        self.editing_finished.emit()