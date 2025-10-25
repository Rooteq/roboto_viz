from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QComboBox, QGroupBox,
                             QMessageBox, QButtonGroup, QInputDialog, QListWidget,
                             QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QPainter, QPen, QBrush, QColor, QFont
from pathlib import Path
import yaml
import shutil
from typing import Optional, Dict, List, Tuple
import json


class CollisionZone:
    """Represents a collision zone with its polygon points and associated routes"""

    def __init__(self, zone_id: int, polygon_points: str, color: QColor,
                 use_polygon_slow: bool = True, use_polygon_stop: bool = False,
                 route_names: List[str] = None):
        self.zone_id = zone_id
        self.polygon_points = polygon_points  # String like "[[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]"
        self.color = color
        self.use_polygon_slow = use_polygon_slow
        self.use_polygon_stop = use_polygon_stop
        self.route_names = route_names or []  # List of route names this zone is associated with

    def to_dict(self):
        return {
            'zone_id': self.zone_id,
            'polygon_points': self.polygon_points,
            'color': [self.color.red(), self.color.green(), self.color.blue()],
            'use_polygon_slow': self.use_polygon_slow,
            'use_polygon_stop': self.use_polygon_stop,
            'route_names': self.route_names
        }

    @staticmethod
    def from_dict(data):
        zone = CollisionZone(
            zone_id=data['zone_id'],
            polygon_points=data['polygon_points'],
            color=QColor(data['color'][0], data['color'][1], data['color'][2]),
            use_polygon_slow=data.get('use_polygon_slow', True),  # Default to True for backward compatibility
            use_polygon_stop=data.get('use_polygon_stop', False),
            route_names=data.get('route_names', [])  # Default to empty list for backward compatibility
        )
        return zone


class CollisionZoneEditor(QWidget):
    """Widget for editing collision monitor zones on PGM maps"""

    collision_zone_updated = pyqtSignal()
    editing_finished = pyqtSignal()

    def __init__(self, route_manager=None, parent=None):
        super().__init__(parent)
        self.route_manager = route_manager
        self.current_map_path: Optional[str] = None
        self.current_yaml_path: Optional[str] = None
        self.original_map_path: Optional[str] = None
        self.collision_mask_pixmap: Optional[QPixmap] = None
        self.drawing_enabled = False
        self.current_tool = 'paint'  # 'paint' or 'erase'
        self.drawing_box = False
        self.box_start_pos = None
        self.box_end_pos = None
        self.map_name = None

        # Collision zone specific data
        self.zones: Dict[int, CollisionZone] = {}  # zone_id -> CollisionZone
        self.next_zone_id = 1
        self.current_zone_id: Optional[int] = None
        self.default_polygon_points: Optional[str] = None  # Default polygon from collision monitor

        # Colors for different zones (cycling through these)
        self.zone_colors = [
            QColor(255, 100, 100),  # Red
            QColor(100, 255, 100),  # Green
            QColor(100, 100, 255),  # Blue
            QColor(255, 255, 100),  # Yellow
            QColor(255, 100, 255),  # Magenta
            QColor(100, 255, 255),  # Cyan
        ]

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
            QListWidget {
                font-size: 14px;
            }
        """)

        # Title
        title = QLabel('Edytor Stref Kolizji')
        title.setStyleSheet('font-weight: bold; font-size: 18px;')
        layout.addWidget(title)

        # Zone management
        zones_group = QGroupBox("Strefy Kolizji")
        zones_layout = QVBoxLayout(zones_group)

        # Zone list
        self.zones_list = QListWidget()
        self.zones_list.setMaximumHeight(150)
        zones_layout.addWidget(self.zones_list)

        # Add zone button
        self.add_zone_btn = QPushButton("Dodaj Nową Strefę")
        self.add_zone_btn.setStyleSheet("""
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
        zones_layout.addWidget(self.add_zone_btn)

        layout.addWidget(zones_group)

        # Tool selection
        tools_group = QGroupBox("Narzędzia")
        tools_layout = QVBoxLayout(tools_group)

        # Tool buttons
        tool_buttons_layout = QHBoxLayout()
        self.tool_group = QButtonGroup()

        self.paint_btn = QPushButton("Maluj Strefę")
        self.paint_btn.setCheckable(True)
        self.paint_btn.setChecked(True)
        self.tool_group.addButton(self.paint_btn, 0)
        tool_buttons_layout.addWidget(self.paint_btn)

        self.erase_btn = QPushButton("Wymaż Strefę")
        self.erase_btn.setCheckable(True)
        self.tool_group.addButton(self.erase_btn, 1)
        tool_buttons_layout.addWidget(self.erase_btn)

        tools_layout.addLayout(tool_buttons_layout)

        layout.addWidget(tools_group)

        # Map operations
        map_ops_group = QGroupBox("Operacje Mapy")
        map_ops_layout = QVBoxLayout(map_ops_group)

        # Clear button
        ops_layout = QHBoxLayout()

        self.clear_zones_btn = QPushButton("Wyczyść Wszystkie Strefy")

        ops_layout.addWidget(self.clear_zones_btn)
        map_ops_layout.addLayout(ops_layout)

        # Save/Cancel buttons
        save_layout = QHBoxLayout()

        self.save_collision_mask_btn = QPushButton("Zapisz Maskę Kolizji")
        self.save_collision_mask_btn.setStyleSheet("""
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

        self.cancel_btn = QPushButton("Anuluj")
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

        save_layout.addWidget(self.save_collision_mask_btn)
        save_layout.addWidget(self.cancel_btn)
        map_ops_layout.addLayout(save_layout)

        layout.addWidget(map_ops_group)

        layout.addStretch()

        self.setup_connections()

    def setup_connections(self):
        self.tool_group.buttonToggled.connect(self.on_tool_changed)
        self.add_zone_btn.clicked.connect(self.add_new_zone)
        self.clear_zones_btn.clicked.connect(self.clear_all_zones)
        self.save_collision_mask_btn.clicked.connect(self.save_collision_mask)
        self.cancel_btn.clicked.connect(self.cancel_editing)
        self.zones_list.itemClicked.connect(self.on_zone_selected)
        self.zones_list.itemDoubleClicked.connect(self.delete_zone)

    def on_tool_changed(self, button, checked):
        if checked:
            if button == self.paint_btn:
                self.current_tool = "paint"
            elif button == self.erase_btn:
                self.current_tool = "erase"

    def add_new_zone(self):
        """Add a new collision zone"""
        # Create custom dialog with checkboxes
        from PyQt5.QtWidgets import QDialog, QTextEdit, QListWidget

        dialog = QDialog(self)
        dialog.setWindowTitle('Nowa Strefa Kolizji')
        dialog.setMinimumWidth(500)
        dialog_layout = QVBoxLayout(dialog)

        # Polygon points input
        label = QLabel('Wprowadź punkty wielokąta:')
        dialog_layout.addWidget(label)

        text_edit = QTextEdit()
        text_edit.setPlainText('[[0.95, 0.55], [0.95, -0.55], [-0.8, -0.55], [-0.8, 0.55]]')
        text_edit.setMaximumHeight(80)
        dialog_layout.addWidget(text_edit)

        # Polygon type checkboxes
        type_label = QLabel('Wybierz typ wielokąta (możesz zaznaczyć oba):')
        dialog_layout.addWidget(type_label)

        slow_checkbox = QCheckBox('PolygonSlow (spowolnienie)')
        slow_checkbox.setChecked(True)  # Default checked
        dialog_layout.addWidget(slow_checkbox)

        stop_checkbox = QCheckBox('PolygonStop (zatrzymanie)')
        stop_checkbox.setChecked(False)
        dialog_layout.addWidget(stop_checkbox)

        # Route selection
        route_label = QLabel('Wybierz trasy dla tej strefy (możesz wybrać wiele):')
        dialog_layout.addWidget(route_label)

        route_list = QListWidget()
        route_list.setSelectionMode(QListWidget.MultiSelection)
        route_list.setMaximumHeight(150)

        # Get available routes from route_manager
        if self.route_manager:
            routes = self.route_manager.load_routes()
            for route_name in sorted(routes.keys()):
                route_list.addItem(route_name)

        dialog_layout.addWidget(route_list)

        # Buttons
        button_layout = QHBoxLayout()
        ok_button = QPushButton('OK')
        cancel_button = QPushButton('Anuluj')
        button_layout.addWidget(ok_button)
        button_layout.addWidget(cancel_button)
        dialog_layout.addLayout(button_layout)

        ok_button.clicked.connect(dialog.accept)
        cancel_button.clicked.connect(dialog.reject)

        if dialog.exec_() != QDialog.Accepted:
            return

        text = text_edit.toPlainText().strip()
        use_polygon_slow = slow_checkbox.isChecked()
        use_polygon_stop = stop_checkbox.isChecked()

        # Get selected routes
        selected_routes = [item.text() for item in route_list.selectedItems()]

        # Validate at least one polygon type is selected
        if not use_polygon_slow and not use_polygon_stop:
            QMessageBox.warning(self, "Błąd", "Musisz wybrać co najmniej jeden typ wielokąta!")
            return

        # Validate at least one route is selected
        if not selected_routes:
            QMessageBox.warning(self, "Błąd", "Musisz wybrać co najmniej jedną trasę!")
            return

        # Validate the input
        try:
            points = json.loads(text)
            if not isinstance(points, list) or len(points) < 3:
                raise ValueError("Wielokąt musi mieć co najmniej 3 punkty")
            for point in points:
                if not isinstance(point, list) or len(point) != 2:
                    raise ValueError("Każdy punkt musi być listą [x, y]")
        except Exception as e:
            QMessageBox.warning(self, "Błąd", f"Nieprawidłowy format punktów: {str(e)}")
            return

        # Create new zone
        zone_id = self.next_zone_id
        self.next_zone_id += 1

        # Get color for this zone (cycle through available colors)
        color = self.zone_colors[(zone_id - 1) % len(self.zone_colors)]

        zone = CollisionZone(zone_id, text, color, use_polygon_slow, use_polygon_stop, selected_routes)
        self.zones[zone_id] = zone

        # Select this zone as current
        self.current_zone_id = zone_id

        # Update the zones list
        self.refresh_zones_list()

        polygon_types = []
        if use_polygon_slow:
            polygon_types.append("Slow")
        if use_polygon_stop:
            polygon_types.append("Stop")
        types_str = " + ".join(polygon_types)

        routes_str = ", ".join(selected_routes)
        QMessageBox.information(self, "Sukces", f"Dodano nową strefę kolizji (ID: {zone_id}, Typ: {types_str}, Trasy: {routes_str})")

    def refresh_zones_list(self):
        """Refresh the zones list display"""
        self.zones_list.clear()
        for zone_id, zone in sorted(self.zones.items()):
            # Display zone with route names
            routes_str = ", ".join(zone.route_names) if zone.route_names else "Brak tras"
            self.zones_list.addItem(f"Strefa {zone_id}: {zone.polygon_points} | Trasy: {routes_str}")

        # Highlight current zone
        if self.current_zone_id is not None:
            for i in range(self.zones_list.count()):
                item = self.zones_list.item(i)
                if item.text().startswith(f"Strefa {self.current_zone_id}:"):
                    self.zones_list.setCurrentItem(item)
                    break

    def on_zone_selected(self, item):
        """Handle zone selection from list"""
        # Extract zone ID from item text
        text = item.text()
        if text.startswith("Strefa "):
            try:
                zone_id = int(text.split(":")[0].replace("Strefa ", ""))
                self.current_zone_id = zone_id
                print(f"DEBUG: Selected collision zone {zone_id}")
            except:
                pass

    def delete_zone(self, item):
        """Delete a zone when double-clicked"""
        # Extract zone ID from item text
        text = item.text()
        if text.startswith("Strefa "):
            try:
                zone_id = int(text.split(":")[0].replace("Strefa ", ""))

                reply = QMessageBox.question(
                    self,
                    'Usuń Strefę',
                    f'Czy na pewno chcesz usunąć strefę {zone_id}?',
                    QMessageBox.Yes | QMessageBox.No
                )

                if reply == QMessageBox.Yes:
                    # Remove zone from map
                    self.erase_zone_from_map(zone_id)

                    # Remove from zones dict
                    if zone_id in self.zones:
                        del self.zones[zone_id]

                    # Clear current selection if this was selected
                    if self.current_zone_id == zone_id:
                        self.current_zone_id = None
                        if self.zones:
                            # Select first available zone
                            self.current_zone_id = min(self.zones.keys())

                    self.refresh_zones_list()
                    self.collision_zone_updated.emit()
                    QMessageBox.information(self, "Sukces", f"Usunięto strefę {zone_id}")
            except Exception as e:
                QMessageBox.warning(self, "Błąd", f"Nie udało się usunąć strefy: {str(e)}")

    def erase_zone_from_map(self, zone_id: int):
        """Erase all pixels of a specific zone from the map"""
        if not self.collision_mask_pixmap or zone_id not in self.zones:
            print(f"DEBUG: Cannot erase - pixmap={self.collision_mask_pixmap is not None}, zone exists={zone_id in self.zones}")
            return

        zone = self.zones[zone_id]
        image = self.collision_mask_pixmap.toImage()

        print(f"DEBUG: Erasing zone {zone_id} with color RGB({zone.color.red()}, {zone.color.green()}, {zone.color.blue()})")

        # Scan entire image and erase pixels matching this zone's color
        pixels_erased = 0
        for y in range(image.height()):
            for x in range(image.width()):
                pixel_color = QColor(image.pixel(x, y))
                if (pixel_color.red() == zone.color.red() and
                    pixel_color.green() == zone.color.green() and
                    pixel_color.blue() == zone.color.blue()):
                    # Erase by painting white
                    image.setPixel(x, y, QColor(255, 255, 255).rgb())
                    pixels_erased += 1

        print(f"DEBUG: Erased {pixels_erased} pixels from zone {zone_id}")

        self.collision_mask_pixmap = QPixmap.fromImage(image)

        # Save the updated pixmap
        try:
            self.collision_mask_pixmap.save(self.current_map_path)
            print(f"DEBUG: Saved map after erasing zone {zone_id} to {self.current_map_path}")
        except Exception as e:
            print(f"ERROR: Error saving pixmap after zone erase: {e}")

    def load_map(self, map_name: str):
        """Load a map for collision zone editing"""
        print(f"DEBUG: CollisionZoneEditor.load_map called with: {map_name}")
        maps_dir = Path.home() / ".robotroutes" / "maps"

        # Try PNG first (color format), fallback to PGM
        collision_map_path = maps_dir / f"collision_{map_name}.png"
        if not collision_map_path.exists():
            collision_map_path = maps_dir / f"collision_{map_name}.pgm"

        collision_yaml_path = maps_dir / f"collision_{map_name}.yaml"
        collision_zones_path = maps_dir / f"collision_{map_name}_zones.json"
        original_map_path = maps_dir / f"{map_name}.pgm"

        print(f"DEBUG: Looking for collision map: {collision_map_path}")
        print(f"DEBUG: Collision map exists: {collision_map_path.exists()}")
        print(f"DEBUG: Collision yaml exists: {collision_yaml_path.exists()}")

        if not collision_map_path.exists() or not collision_yaml_path.exists():
            print(f"DEBUG: Collision map files not found for '{map_name}'")
            QMessageBox.warning(self, "Błąd", f"Pliki mapy kolizji dla '{map_name}' nie zostały znalezione!")
            return False

        # Store paths
        self.current_map_path = str(collision_map_path)
        self.current_yaml_path = str(collision_yaml_path)
        self.original_map_path = str(original_map_path)
        self.map_name = map_name

        # Load zones from all route-specific JSON files
        # Pattern: collision_<route_name>_zones.json
        self.zones = {}
        zone_files = list(maps_dir.glob("collision_*_zones.json"))

        for zone_file in zone_files:
            # Extract route name from filename
            # collision_<route_name>_zones.json
            filename = zone_file.stem  # collision_<route_name>_zones
            if filename.endswith("_zones"):
                try:
                    with open(zone_file, 'r') as f:
                        zones_data = json.load(f)
                        for zone_id_str, zone_dict in zones_data.items():
                            zone_id = int(zone_id_str)
                            # If zone already exists, merge route names
                            if zone_id in self.zones:
                                # Merge route names from this file
                                existing_zone = self.zones[zone_id]
                                new_routes = zone_dict.get('route_names', [])
                                for route in new_routes:
                                    if route not in existing_zone.route_names:
                                        existing_zone.route_names.append(route)
                            else:
                                # Create new zone
                                self.zones[zone_id] = CollisionZone.from_dict(zone_dict)
                    print(f"DEBUG: Loaded zones from {zone_file.name}")
                except Exception as e:
                    print(f"Error loading zones from {zone_file}: {e}")

        if self.zones:
            self.next_zone_id = max(self.zones.keys()) + 1
            self.current_zone_id = min(self.zones.keys())
            print(f"DEBUG: Loaded {len(self.zones)} total zones from route files")
        else:
            self.zones = {}
            print(f"DEBUG: No zones loaded")

        # Load the collision_ prefixed map for editing
        self.collision_mask_pixmap = QPixmap(self.current_map_path)
        self.drawing_enabled = True

        # Refresh zones list
        self.refresh_zones_list()

        return True

    def get_display_map_path(self):
        """Get the path to the current working map (for display purposes)"""
        return self.current_map_path if self.current_map_path else None

    def clear_all_zones(self):
        """Clear all collision zones by loading the original map"""
        reply = QMessageBox.question(self, 'Wyczyść Strefy',
                                   'Czy na pewno chcesz wyczyścić wszystkie strefy kolizji?',
                                   QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.original_map_path and self.current_map_path:
                try:
                    # Copy original to collision_ prefixed map
                    shutil.copy2(self.original_map_path, self.current_map_path)
                    self.collision_mask_pixmap = QPixmap(self.current_map_path)

                    # Clear all zones
                    self.zones = {}
                    self.current_zone_id = None
                    self.next_zone_id = 1
                    self.refresh_zones_list()

                    QMessageBox.information(self, "Sukces", "Wszystkie strefy kolizji zostały wyczyszczone!")
                    self.collision_zone_updated.emit()
                except Exception as e:
                    QMessageBox.critical(self, "Błąd", f"Nie udało się wyczyścić stref kolizji: {str(e)}")

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
        print(f"DEBUG: drawing_box={self.drawing_box}, drawing_enabled={self.drawing_enabled}, pixmap={self.collision_mask_pixmap is not None}")

        if not self.drawing_box or not self.drawing_enabled or not self.collision_mask_pixmap:
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
        max_x = min(self.collision_mask_pixmap.width() - 1, max_x)
        max_y = min(self.collision_mask_pixmap.height() - 1, max_y)

        if min_x >= max_x or min_y >= max_y:
            self.drawing_box = False
            return

        # Create painter for the pixmap
        painter = QPainter(self.collision_mask_pixmap)

        if self.current_tool == 'paint':
            # Paint collision zone
            if self.current_zone_id is None:
                QMessageBox.warning(self, "Błąd", "Proszę najpierw wybrać lub utworzyć strefę!")
                self.drawing_box = False
                painter.end()
                return

            zone = self.zones[self.current_zone_id]
            color = zone.color
        else:
            # Erase (paint with white/free space)
            color = QColor(255, 255, 255)  # White for free space

        painter.fillRect(min_x, min_y, max_x - min_x, max_y - min_y, color)
        painter.end()

        # Save the updated pixmap back to file
        try:
            self.collision_mask_pixmap.save(self.current_map_path)
            print(f"DEBUG: Saved collision zone changes to {self.current_map_path}")
            self.collision_zone_updated.emit()
        except Exception as e:
            print(f"Error saving pixmap: {e}")

        self.drawing_box = False

    def cancel_box_drawing(self):
        """Cancel current box drawing"""
        self.drawing_box = False
        self.box_start_pos = None
        self.box_end_pos = None

    def save_collision_mask(self):
        """Save the current collision zones - zones are saved per-route"""
        if not self.current_map_path or not self.current_yaml_path or not self.map_name:
            QMessageBox.warning(self, "Błąd", "Nie załadowano mapy!")
            return

        try:
            # Get map directory
            map_path = Path(self.current_map_path)
            maps_dir = map_path.parent

            # Use collision_ prefixed naming - SAVE AS PNG to preserve colors!
            collision_map_png = maps_dir / f"collision_{self.map_name}.png"
            collision_map_yaml = maps_dir / f"collision_{self.map_name}.yaml"

            # Save current working map as PNG (to preserve RGB colors)
            self.collision_mask_pixmap.save(str(collision_map_png), "PNG")

            # Read the original map's YAML to get base properties
            original_yaml_path = maps_dir / f"{self.map_name}.yaml"
            if original_yaml_path.exists():
                with open(original_yaml_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
            else:
                # Fallback to current yaml if original doesn't exist
                with open(self.current_yaml_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)

            # Update YAML - point to PNG file instead of PGM
            yaml_data['image'] = f'collision_{self.map_name}.png'

            # Write collision map YAML
            with open(collision_map_yaml, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)

            # Save zones per-route: create a separate JSON file for each route
            saved_files = []

            # Group zones by route
            route_zones = {}  # route_name -> list of zones
            for zone_id, zone in self.zones.items():
                for route_name in zone.route_names:
                    if route_name not in route_zones:
                        route_zones[route_name] = {}
                    route_zones[route_name][str(zone_id)] = zone.to_dict()

            # Save a file for each route
            for route_name, zones_data in route_zones.items():
                collision_zones_json = maps_dir / f"collision_{route_name}_zones.json"
                with open(collision_zones_json, 'w') as f:
                    json.dump(zones_data, f, indent=2)
                saved_files.append(str(collision_zones_json))

            files_list = "\n".join(saved_files) if saved_files else "Brak stref do zapisania"
            QMessageBox.information(self, "Sukces",
                                  f"Maska kolizji zapisana jako:\n{collision_map_png}\n{collision_map_yaml}\n\nStrefy zapisane dla tras:\n{files_list}")

        except Exception as e:
            QMessageBox.critical(self, "Błąd", f"Nie udało się zapisać maski kolizji: {str(e)}")

    def cancel_editing(self):
        """Cancel collision zone editing and return to original map"""
        reply = QMessageBox.question(self, 'Anuluj Edycję',
                                   'Anulować edycję stref kolizji? Wszelkie niezapisane zmiany zostaną utracone.',
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
        self.collision_mask_pixmap = None

        # Clear zones
        self.zones = {}
        self.current_zone_id = None
        self.next_zone_id = 1
        self.refresh_zones_list()

        # Clear file paths
        self.current_map_path = None
        self.original_map_path = None
        self.current_yaml_path = None
        self.map_name = None

        self.editing_finished.emit()

    def get_zone_at_position(self, scene_x: float, scene_y: float) -> Optional[int]:
        """Get the zone ID at a specific position on the map"""
        if not self.collision_mask_pixmap:
            return None

        x, y = int(scene_x), int(scene_y)
        if x < 0 or y < 0 or x >= self.collision_mask_pixmap.width() or y >= self.collision_mask_pixmap.height():
            return None

        image = self.collision_mask_pixmap.toImage()
        pixel_color = QColor(image.pixel(x, y))

        # Find zone with matching color
        for zone_id, zone in self.zones.items():
            if (pixel_color.red() == zone.color.red() and
                pixel_color.green() == zone.color.green() and
                pixel_color.blue() == zone.color.blue()):
                return zone_id

        return None
