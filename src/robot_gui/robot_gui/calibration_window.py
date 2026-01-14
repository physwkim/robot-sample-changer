"""Calibration Window with 2D Coordinate Visualization and YAML Editor using silx."""

import os
import math
import yaml
from silx.gui import qt
from silx.gui.widgets.WaitingPushButton import WaitingPushButton


class CoordinateView2D(qt.QWidget):
    """2D visualization of TCP coordinate frame (End-Effector Local Frame).
    
    End-Effector Local Frame:
    - X axis: Left direction (로봇을 바라볼 때 왼쪽) - RED
    - Y axis: Down direction (아래 방향) - GREEN  
    - Z axis: Forward direction (로봇 정면 방향, 진행 방향) - BLUE
    
    Shows two views: Front (XY) and Top (XZ)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(380, 260)
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._offset_z = 0.0

    def set_offsets(self, x_mm, y_mm, z_mm):
        """Set current offset values (in mm)."""
        self._offset_x = x_mm
        self._offset_y = y_mm
        self._offset_z = z_mm
        self.update()

    def paintEvent(self, event):
        from silx.gui.qt import QPainter, QPen, QBrush, QColor, QFont, QPolygonF, QPointF

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(25, 25, 35))

        w = self.width()
        h = self.height()

        # Draw two views side by side
        view_width = w // 2 - 20
        view_height = h - 45

        # Front View (XY plane) - Left side
        self._draw_front_view(painter, 10, 32, view_width, view_height)

        # Top View (XZ plane) - Right side  
        self._draw_top_view(painter, w // 2 + 10, 32, view_width, view_height)

        # Title
        painter.setPen(QPen(QColor(200, 200, 200)))
        font = QFont("Arial", 10, QFont.Bold)
        painter.setFont(font)
        painter.drawText(10, 20, "End-Effector Local Frame (TCP)")

    def _draw_arrow(self, painter, x1, y1, x2, y2, color, width):
        """Draw an arrow from (x1,y1) to (x2,y2)."""
        from silx.gui.qt import QPen, QBrush, QColor, QPolygonF, QPointF

        painter.setPen(QPen(color, width))
        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Arrow head
        angle = math.atan2(y2 - y1, x2 - x1)
        arrow_size = 10

        p1 = QPointF(
            x2 - arrow_size * math.cos(angle - math.pi / 6),
            y2 - arrow_size * math.sin(angle - math.pi / 6)
        )
        p2 = QPointF(
            x2 - arrow_size * math.cos(angle + math.pi / 6),
            y2 - arrow_size * math.sin(angle + math.pi / 6)
        )

        painter.setBrush(QBrush(color))
        arrow = QPolygonF([QPointF(x2, y2), p1, p2])
        painter.drawPolygon(arrow)

    def _draw_front_view(self, painter, x, y, w, h):
        """Draw front view (XY plane) - looking at robot from front."""
        from silx.gui.qt import QPen, QBrush, QColor, QFont

        # Background
        painter.fillRect(x, y, w, h, QColor(35, 35, 45))
        painter.setPen(QPen(QColor(80, 80, 100), 2))
        painter.drawRect(x, y, w, h)

        # Title
        painter.setPen(QPen(QColor(180, 180, 200)))
        font = QFont("Arial", 9, QFont.Bold)
        painter.setFont(font)
        painter.drawText(x + 8, y + 16, "Front View (XY)")
        
        small_font = QFont("Arial", 8)
        painter.setFont(small_font)
        painter.setPen(QPen(QColor(140, 140, 160)))
        painter.drawText(x + 8, y + 28, "로봇 정면에서 본 모습")

        # Origin
        cx = x + w // 2
        cy = y + h // 2 + 12

        axis_len = min(w, h) // 3

        # X axis (Left, screen left) - RED
        self._draw_arrow(painter, cx, cy, cx - axis_len, cy, QColor(255, 80, 80), 3)
        painter.setPen(QPen(QColor(255, 100, 100)))
        font = QFont("Arial", 9, QFont.Bold)
        painter.setFont(font)
        painter.drawText(cx - axis_len - 20, cy + 4, "X")
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        painter.drawText(cx - axis_len - 28, cy + 15, "←왼쪽")

        # Y axis (Down) - GREEN
        self._draw_arrow(painter, cx, cy, cx, cy + axis_len, QColor(80, 255, 80), 3)
        painter.setPen(QPen(QColor(100, 255, 100)))
        font = QFont("Arial", 9, QFont.Bold)
        painter.setFont(font)
        painter.drawText(cx + 6, cy + axis_len + 4, "Y")
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        painter.drawText(cx + 6, cy + axis_len + 15, "↓아래")

        # Z indicator (into screen - circle with X)
        painter.setPen(QPen(QColor(80, 150, 255), 2))
        painter.setBrush(QBrush(QColor(40, 80, 140)))
        painter.drawEllipse(cx - 10, cy - 10, 20, 20)
        painter.setPen(QPen(QColor(120, 180, 255), 2))
        painter.drawLine(cx - 5, cy - 5, cx + 5, cy + 5)
        painter.drawLine(cx - 5, cy + 5, cx + 5, cy - 5)
        
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        painter.setPen(QPen(QColor(120, 180, 255)))
        painter.drawText(cx + 14, cy - 10, "Z(정면)")
        painter.drawText(cx + 14, cy + 2, "⊗진행")

        # Origin dot
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(cx - 3, cy - 3, 6, 6)

        # Offset indicator
        if abs(self._offset_x) > 0.01 or abs(self._offset_y) > 0.01:
            scale = axis_len / 5.0  # 5mm = full axis
            ox = cx - self._offset_x * scale
            oy = cy + self._offset_y * scale
            painter.setPen(QPen(QColor(255, 200, 50), 2, qt.Qt.DashLine))
            painter.drawLine(int(cx), int(cy), int(ox), int(oy))
            painter.setBrush(QBrush(QColor(255, 200, 50)))
            painter.drawEllipse(int(ox) - 4, int(oy) - 4, 8, 8)

    def _draw_top_view(self, painter, x, y, w, h):
        """Draw top view (XZ plane) - looking down from above (behind the robot)."""
        from silx.gui.qt import QPen, QBrush, QColor, QFont

        # Background
        painter.fillRect(x, y, w, h, QColor(35, 35, 45))
        painter.setPen(QPen(QColor(80, 80, 100), 2))
        painter.drawRect(x, y, w, h)

        # Title
        painter.setPen(QPen(QColor(180, 180, 200)))
        font = QFont("Arial", 9, QFont.Bold)
        painter.setFont(font)
        painter.drawText(x + 8, y + 16, "Top View (XZ)")
        
        small_font = QFont("Arial", 8)
        painter.setFont(small_font)
        painter.setPen(QPen(QColor(140, 140, 160)))
        painter.drawText(x + 8, y + 28, "위에서 (로봇 뒤쪽에서)")

        # Origin
        cx = x + w // 2
        cy = y + h // 2 + 12

        axis_len = min(w, h) // 3

        # X axis (Left from robot's perspective = screen RIGHT when viewed from behind)
        self._draw_arrow(painter, cx, cy, cx + axis_len, cy, QColor(255, 80, 80), 3)
        painter.setPen(QPen(QColor(255, 100, 100)))
        font = QFont("Arial", 9, QFont.Bold)
        painter.setFont(font)
        painter.drawText(cx + axis_len + 5, cy + 4, "X")
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        painter.drawText(cx + axis_len + 5, cy + 15, "왼쪽→")

        # Z axis (Forward) - BLUE
        self._draw_arrow(painter, cx, cy, cx, cy - axis_len, QColor(80, 150, 255), 3)
        painter.setPen(QPen(QColor(120, 180, 255)))
        font = QFont("Arial", 9, QFont.Bold)
        painter.setFont(font)
        painter.drawText(cx + 6, cy - axis_len - 4, "Z")
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        painter.drawText(cx + 6, cy - axis_len + 7, "↑정면")

        # Y indicator (into screen/down - circle with dot)
        painter.setPen(QPen(QColor(80, 255, 80), 2))
        painter.setBrush(QBrush(QColor(40, 120, 60)))
        painter.drawEllipse(cx - 10, cy - 10, 20, 20)
        painter.setBrush(QBrush(QColor(120, 255, 120)))
        painter.drawEllipse(cx - 3, cy - 3, 6, 6)
        
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        painter.setPen(QPen(QColor(120, 255, 120)))
        painter.drawText(cx - 45, cy - 10, "Y(아래)")
        painter.drawText(cx - 55, cy + 2, "⊙종이안쪽")

        # Origin dot
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(cx - 3, cy - 3, 6, 6)

        # Offset indicator (X flipped for top view from behind)
        if abs(self._offset_x) > 0.01 or abs(self._offset_z) > 0.01:
            scale = axis_len / 5.0
            ox = cx + self._offset_x * scale  # Flipped for behind view
            oz = cy - self._offset_z * scale
            painter.setPen(QPen(QColor(255, 200, 50), 2, qt.Qt.DashLine))
            painter.drawLine(int(cx), int(cy), int(ox), int(oz))
            painter.setBrush(QBrush(QColor(255, 200, 50)))
            painter.drawEllipse(int(ox) - 4, int(oz) - 4, 8, 8)


class CoordinateLegend(qt.QWidget):
    """Legend explaining coordinate axes."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = qt.QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(15)

        x_label = qt.QLabel("● X: 왼쪽")
        x_label.setStyleSheet("color: #FF5555; font-size: 10px; font-weight: bold;")
        layout.addWidget(x_label)

        y_label = qt.QLabel("● Y: 아래")
        y_label.setStyleSheet("color: #55FF55; font-size: 10px; font-weight: bold;")
        layout.addWidget(y_label)

        z_label = qt.QLabel("● Z: 정면")
        z_label.setStyleSheet("color: #5599FF; font-size: 10px; font-weight: bold;")
        layout.addWidget(z_label)

        layout.addStretch()


class OffsetTableWidget(qt.QWidget):
    """Table-based offset editor for all holders."""

    offset_changed = qt.Signal()  # Emitted when any offset changes

    def __init__(self, parent=None):
        super().__init__(parent)
        self._params = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # Create table
        self.table = qt.QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Target", "X (mm)", "Y (mm)", "Z (mm)"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(qt.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #3a3a3a;
                alternate-background-color: #424242;
                gridline-color: #555;
                font-size: 10px;
            }
            QTableWidget::item {
                padding: 4px;
                color: #ddd;
            }
            QTableWidget::item:selected {
                background-color: #4a6a90;
            }
            QHeaderView::section {
                background-color: #4a4a4a;
                color: #eee;
                padding: 4px;
                border: 1px solid #555;
                font-weight: bold;
            }
        """)

        # Rows: Sample Holder, Holder 1, Holder 2-10 (multi offsets)
        row_labels = ["Sample Holder", "Holder 1 (base)"] + [f"Holder {i} (add)" for i in range(2, 11)]
        self.table.setRowCount(len(row_labels))

        for row, label in enumerate(row_labels):
            # Target name (read-only)
            name_item = qt.QTableWidgetItem(label)
            name_item.setFlags(name_item.flags() & ~qt.Qt.ItemIsEditable)
            if row == 0:
                name_item.setBackground(qt.QColor(75, 55, 95))  # Sample Holder - purple tint
            elif row == 1:
                name_item.setBackground(qt.QColor(55, 75, 95))  # Holder 1 - blue tint
            else:
                name_item.setBackground(qt.QColor(65, 65, 75))  # Holder 2-10 - gray
            self.table.setItem(row, 0, name_item)

            # X, Y, Z spinboxes
            for col in range(1, 4):
                spin = qt.QDoubleSpinBox()
                spin.setRange(-50.0, 50.0)
                spin.setSingleStep(0.1)
                spin.setDecimals(3)
                spin.setSuffix("")
                spin.setButtonSymbols(qt.QAbstractSpinBox.NoButtons)
                spin.setAlignment(qt.Qt.AlignRight)
                
                # Color coding
                if col == 1:  # X
                    spin.setStyleSheet("QDoubleSpinBox { color: #FF8888; background: #454545; border: 1px solid #555; }")
                elif col == 2:  # Y
                    spin.setStyleSheet("QDoubleSpinBox { color: #88FF88; background: #454545; border: 1px solid #555; }")
                else:  # Z
                    spin.setStyleSheet("QDoubleSpinBox { color: #88AAFF; background: #454545; border: 1px solid #555; }")

                # Holder 2-10 don't have Y offset
                if row >= 2 and col == 2:
                    spin.setEnabled(False)
                    spin.setStyleSheet("QDoubleSpinBox { color: #666; background: #3a3a3a; border: 1px solid #444; }")

                spin.valueChanged.connect(self._on_value_changed)
                self.table.setCellWidget(row, col, spin)

        self.table.setRowHeight(0, 28)
        for row in range(1, self.table.rowCount()):
            self.table.setRowHeight(row, 26)

        layout.addWidget(self.table)

    def set_params(self, params):
        """Load parameters into table."""
        self._params = params

        # Block signals during update
        self.table.blockSignals(True)

        # Sample Holder (row 0)
        self._set_cell_value(0, 1, params.get('sample_holder_on_position_x_offset', 0) * 1000)
        self._set_cell_value(0, 2, params.get('sample_holder_on_position_y_offset', 0) * 1000)
        self._set_cell_value(0, 3, params.get('sample_holder_on_position_z_offset', 0) * 1000)

        # Holder 1 (row 1)
        self._set_cell_value(1, 1, params.get('holder1_on_position_x_offset', 0) * 1000)
        self._set_cell_value(1, 2, params.get('holder1_on_position_y_offset', 0) * 1000)
        self._set_cell_value(1, 3, params.get('holder1_on_position_z_offset', 0) * 1000)

        # Holder 2-10 (rows 2-10)
        x_offsets = params.get('holder_multi_x_offsets', [0] * 9)
        z_offsets = params.get('holder_multi_z_offsets', [0] * 9)

        for i in range(9):
            row = i + 2
            if i < len(x_offsets):
                self._set_cell_value(row, 1, x_offsets[i] * 1000)
            if i < len(z_offsets):
                self._set_cell_value(row, 3, z_offsets[i] * 1000)

        self.table.blockSignals(False)

    def _set_cell_value(self, row, col, value):
        """Set spinbox value at row, col."""
        widget = self.table.cellWidget(row, col)
        if widget:
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)

    def _get_cell_value(self, row, col):
        """Get spinbox value at row, col (in mm)."""
        widget = self.table.cellWidget(row, col)
        if widget:
            return widget.value()
        return 0.0

    def _on_value_changed(self):
        """Handle any value change."""
        self.offset_changed.emit()

    def get_updated_params(self):
        """Get updated parameters from table."""
        params = dict(self._params)

        # Sample Holder
        params['sample_holder_on_position_x_offset'] = self._get_cell_value(0, 1) / 1000.0
        params['sample_holder_on_position_y_offset'] = self._get_cell_value(0, 2) / 1000.0
        params['sample_holder_on_position_z_offset'] = self._get_cell_value(0, 3) / 1000.0

        # Holder 1
        params['holder1_on_position_x_offset'] = self._get_cell_value(1, 1) / 1000.0
        params['holder1_on_position_y_offset'] = self._get_cell_value(1, 2) / 1000.0
        params['holder1_on_position_z_offset'] = self._get_cell_value(1, 3) / 1000.0

        # Holder 2-10
        x_offsets = []
        z_offsets = []
        for i in range(9):
            row = i + 2
            x_offsets.append(self._get_cell_value(row, 1) / 1000.0)
            z_offsets.append(self._get_cell_value(row, 3) / 1000.0)

        params['holder_multi_x_offsets'] = x_offsets
        params['holder_multi_z_offsets'] = z_offsets

        return params

    def get_selected_offsets_mm(self):
        """Get offsets for currently selected row."""
        row = self.table.currentRow()
        if row < 0:
            row = 0
        return (
            self._get_cell_value(row, 1),
            self._get_cell_value(row, 2),
            self._get_cell_value(row, 3)
        )


class YamlOffsetEditor(qt.QGroupBox):
    """YAML offset editor with table view."""

    offset_changed = qt.Signal(float, float, float)  # x, y, z in mm

    def __init__(self, parent=None):
        super().__init__("Offset Editor (taught_waypoints.yaml)", parent)
        self._yaml_path = ""
        self._yaml_data = {}
        self._params = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(6)

        # File controls
        file_layout = qt.QHBoxLayout()
        self.path_label = qt.QLabel("No file loaded")
        self.path_label.setStyleSheet("color: #888; font-size: 9px;")
        self.path_label.setWordWrap(True)
        file_layout.addWidget(self.path_label, 1)
        layout.addLayout(file_layout)

        btn_layout = qt.QHBoxLayout()
        self.load_btn = qt.QPushButton("Load YAML")
        self.load_btn.setMinimumHeight(30)
        self.load_btn.clicked.connect(self._load_yaml)
        btn_layout.addWidget(self.load_btn)

        self.save_btn = qt.QPushButton("Save YAML")
        self.save_btn.setMinimumHeight(30)
        self.save_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self.save_btn.clicked.connect(self._save_yaml)
        self.save_btn.setEnabled(False)
        btn_layout.addWidget(self.save_btn)
        layout.addLayout(btn_layout)

        # Table
        self.offset_table = OffsetTableWidget()
        self.offset_table.offset_changed.connect(self._on_offset_changed)
        self.offset_table.table.itemSelectionChanged.connect(self._on_selection_changed)
        layout.addWidget(self.offset_table)

    def _load_yaml(self):
        default_path = os.path.expanduser("~/ws/src/epics_robot/config")
        path, _ = qt.QFileDialog.getOpenFileName(
            self, "Load Waypoints YAML", default_path, "YAML Files (*.yaml *.yml)"
        )
        if path:
            self.load_yaml_file(path)

    def load_yaml_file(self, path):
        try:
            with open(path, 'r') as f:
                self._yaml_data = yaml.safe_load(f)

            self._yaml_path = path
            self.path_label.setText(os.path.basename(path))
            self.path_label.setStyleSheet("color: #4CAF50; font-size: 9px;")
            self.save_btn.setEnabled(True)

            if '/**' in self._yaml_data and 'ros__parameters' in self._yaml_data['/**']:
                self._params = self._yaml_data['/**']['ros__parameters']
            elif 'ros__parameters' in self._yaml_data:
                self._params = self._yaml_data['ros__parameters']
            else:
                self._params = self._yaml_data

            self.offset_table.set_params(self._params)
            self._emit_current_offset()

        except Exception as e:
            qt.QMessageBox.critical(self, "Error", f"Failed to load YAML:\n{e}")

    def _on_offset_changed(self):
        """Handle table value change."""
        self._mark_modified()
        self._emit_current_offset()

    def _on_selection_changed(self):
        """Handle row selection change."""
        self._emit_current_offset()

    def _emit_current_offset(self):
        """Emit offset for currently selected row."""
        x, y, z = self.offset_table.get_selected_offsets_mm()
        self.offset_changed.emit(x, y, z)

    def _mark_modified(self):
        if self._yaml_path and not self.path_label.text().endswith(" *"):
            self.path_label.setText(os.path.basename(self._yaml_path) + " *")
            self.path_label.setStyleSheet("color: #FFA500; font-size: 9px;")

    def _save_yaml(self):
        if not self._yaml_path:
            return

        # Update params from table
        updated_params = self.offset_table.get_updated_params()

        # Merge back to yaml data
        if '/**' in self._yaml_data and 'ros__parameters' in self._yaml_data['/**']:
            self._yaml_data['/**']['ros__parameters'].update(updated_params)
        elif 'ros__parameters' in self._yaml_data:
            self._yaml_data['ros__parameters'].update(updated_params)
        else:
            self._yaml_data.update(updated_params)

        try:
            with open(self._yaml_path, 'w') as f:
                yaml.dump(self._yaml_data, f, default_flow_style=None, allow_unicode=True, sort_keys=False)

            self.path_label.setText(os.path.basename(self._yaml_path))
            self.path_label.setStyleSheet("color: #4CAF50; font-size: 9px;")
            qt.QMessageBox.information(self, "Saved", "YAML saved!\n다음 로봇 트리거 시 적용됩니다.")

        except Exception as e:
            qt.QMessageBox.critical(self, "Error", f"Failed to save YAML:\n{e}")


class CalibrationControls(qt.QGroupBox):
    """Calibration mode controls."""

    holder_calib_requested = qt.Signal(int)
    sample_calib_requested = qt.Signal()
    continue_requested = qt.Signal()
    abort_requested = qt.Signal()

    def __init__(self, parent=None):
        super().__init__("Calibration Controls", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(8)

        holder_layout = qt.QHBoxLayout()
        holder_layout.addWidget(qt.QLabel("Holder:"))
        self.holder_combo = qt.QComboBox()
        for i in range(1, 11):
            self.holder_combo.addItem(f"Holder {i}", i)
        holder_layout.addWidget(self.holder_combo)
        layout.addLayout(holder_layout)

        btn_layout = qt.QHBoxLayout()

        self.holder_calib_btn = WaitingPushButton("Holder Calib")
        self.holder_calib_btn.setMinimumHeight(38)
        self.holder_calib_btn.setStyleSheet("""
            QPushButton { background-color: #9C27B0; color: white; border-radius: 5px; font-weight: bold; font-size: 11px; }
            QPushButton:hover { background-color: #7B1FA2; }
        """)
        self.holder_calib_btn.clicked.connect(
            lambda: self.holder_calib_requested.emit(self.holder_combo.currentData())
        )
        btn_layout.addWidget(self.holder_calib_btn)

        self.sample_calib_btn = WaitingPushButton("Sample Calib")
        self.sample_calib_btn.setMinimumHeight(38)
        self.sample_calib_btn.setStyleSheet("""
            QPushButton { background-color: #673AB7; color: white; border-radius: 5px; font-weight: bold; font-size: 11px; }
            QPushButton:hover { background-color: #512DA8; }
        """)
        self.sample_calib_btn.clicked.connect(self.sample_calib_requested.emit)
        btn_layout.addWidget(self.sample_calib_btn)

        layout.addLayout(btn_layout)

        action_layout = qt.QHBoxLayout()

        self.continue_btn = qt.QPushButton("Continue")
        self.continue_btn.setMinimumHeight(35)
        self.continue_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; border-radius: 5px; font-weight: bold; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self.continue_btn.setEnabled(False)
        self.continue_btn.clicked.connect(self.continue_requested.emit)
        action_layout.addWidget(self.continue_btn)

        self.abort_btn = qt.QPushButton("Abort")
        self.abort_btn.setMinimumHeight(35)
        self.abort_btn.setStyleSheet("""
            QPushButton { background-color: #f44336; color: white; border-radius: 5px; font-weight: bold; }
            QPushButton:hover { background-color: #da190b; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self.abort_btn.setEnabled(False)
        self.abort_btn.clicked.connect(self.abort_requested.emit)
        action_layout.addWidget(self.abort_btn)

        layout.addLayout(action_layout)

    def set_calibrating(self, calibrating):
        self.holder_calib_btn.setEnabled(not calibrating)
        self.sample_calib_btn.setEnabled(not calibrating)
        self.holder_combo.setEnabled(not calibrating)
        self.continue_btn.setEnabled(calibrating)
        self.abort_btn.setEnabled(calibrating)


class CalibrationWindow(qt.QDialog):
    """Main calibration window with 2D TCP visualization and YAML editor."""

    def __init__(self, epics_handler, parent=None):
        super().__init__(parent)
        self.epics_handler = epics_handler
        self._calibrating = False
        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        self.setWindowTitle("Calibration - TCP Coordinate & YAML Editor")
        self.setMinimumSize(950, 600)

        layout = qt.QHBoxLayout(self)
        layout.setSpacing(10)

        # Left: 2D visualization + legend + status
        left_widget = qt.QWidget()
        left_layout = qt.QVBoxLayout(left_widget)
        left_layout.setSpacing(6)

        self.coord_view = CoordinateView2D()
        left_layout.addWidget(self.coord_view)

        self.legend = CoordinateLegend()
        left_layout.addWidget(self.legend)

        # Calibration controls
        self.calib_controls = CalibrationControls()
        left_layout.addWidget(self.calib_controls)

        # Status
        status_group = qt.QGroupBox("Status")
        status_layout = qt.QVBoxLayout(status_group)
        self.status_label = qt.QLabel("Ready\n\nLoad YAML to edit offsets")
        font = qt.QFont()
        font.setPointSize(9)
        self.status_label.setFont(font)
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #aaa;")
        status_layout.addWidget(self.status_label)
        left_layout.addWidget(status_group)

        layout.addWidget(left_widget, stretch=1)

        # Right: YAML editor with table
        self.yaml_editor = YamlOffsetEditor()
        layout.addWidget(self.yaml_editor, stretch=1)

    def _connect_signals(self):
        self.calib_controls.holder_calib_requested.connect(self._start_holder_calib)
        self.calib_controls.sample_calib_requested.connect(self._start_sample_calib)
        self.calib_controls.continue_requested.connect(self._continue_calib)
        self.calib_controls.abort_requested.connect(self._abort_calib)

        self.yaml_editor.offset_changed.connect(self._on_offset_changed)

    def _on_offset_changed(self, x_mm, y_mm, z_mm):
        """Update coordinate view when offset changes."""
        self.coord_view.set_offsets(x_mm, y_mm, z_mm)

    def _start_holder_calib(self, holder_num):
        self.epics_handler.set_holder(holder_num)
        self.epics_handler.set_calib_mode(1)
        self.epics_handler.trigger_sequence()

        self._calibrating = True
        self.calib_controls.set_calibrating(True)
        self.status_label.setText(
            f"Holder {holder_num} Calibration\n\n"
            "로봇 이동 중...\n\n"
            "정렬 확인 → 테이블에서 오프셋 조정\n"
            "→ Save YAML → Continue"
        )
        self.status_label.setStyleSheet("color: #9C27B0;")

    def _start_sample_calib(self):
        holder_num = self.calib_controls.holder_combo.currentData()
        self.epics_handler.set_holder(holder_num)
        self.epics_handler.set_calib_mode(2)
        self.epics_handler.trigger_sequence()

        self._calibrating = True
        self.calib_controls.set_calibrating(True)
        self.status_label.setText(
            f"Sample Holder Calibration\n(from Holder {holder_num})\n\n"
            "로봇 이동 중...\n\n"
            "정렬 확인 → 테이블에서 오프셋 조정\n"
            "→ Save YAML → Continue"
        )
        self.status_label.setStyleSheet("color: #673AB7;")

    def _continue_calib(self):
        self.epics_handler.trigger_sequence()
        self._calibrating = False
        self.calib_controls.set_calibrating(False)
        self.status_label.setText("샘플 복귀 중...")
        self.status_label.setStyleSheet("color: #4CAF50;")

    def _abort_calib(self):
        self.epics_handler.set_wait(2)
        self.epics_handler.trigger_sequence()
        self._calibrating = False
        self.calib_controls.set_calibrating(False)
        self.status_label.setText("캘리브레이션 중단\n샘플 복귀 중...")
        self.status_label.setStyleSheet("color: #f44336;")

    def load_yaml(self, path):
        """Load YAML file programmatically."""
        self.yaml_editor.load_yaml_file(path)
