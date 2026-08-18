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



def _round7(v):
    """The write grain shared with the daemon's persist: 7 decimals."""
    return round(float(v), 7)


def _set_scalar_line(text, key, value):
    """Replace the value on the single `key:` line, keeping everything else."""
    prefix = key + ':'
    out, hit = [], False
    for line in text.splitlines():
        stripped = line.lstrip()
        if stripped.startswith(prefix):
            if hit:
                raise ValueError(f"'{key}' appears more than once")
            float(stripped[len(prefix):].strip())  # must already be a scalar
            indent = line[:len(line) - len(stripped)]
            out.append(f"{indent}{key}: {value!r}")
            hit = True
            continue
        out.append(line)
    if not hit:
        raise ValueError(f"key '{key}' not found")
    return "\n".join(out) + "\n"


def _set_list_entry(text, key, index, value):
    """Replace one entry of the flow list at `key:`, re-emitting it on one
    line (wrapped continuations are consumed up to the closing bracket)."""
    prefix = key + ':'
    src = text.splitlines()
    out, hit, i = [], False, 0
    while i < len(src):
        line = src[i]
        stripped = line.lstrip()
        if not stripped.startswith(prefix):
            out.append(line)
            i += 1
            continue
        if hit:
            raise ValueError(f"'{key}' appears more than once")
        indent = line[:len(line) - len(stripped)]
        body = stripped[len(prefix):].strip()
        while not body.endswith(']'):
            i += 1
            if i >= len(src):
                raise ValueError(f"list '{key}' is not closed with ']'")
            body += ' ' + src[i].strip()
        if not (body.startswith('[') and body.endswith(']')):
            raise ValueError(f"'{key}' is not a flow list")
        values = [float(part) for part in body[1:-1].split(',')]
        if index >= len(values):
            raise ValueError(f"'{key}' has {len(values)} entries, wanted index {index}")
        values[index] = value
        joined = ", ".join(repr(v) for v in values)
        out.append(f"{indent}{key}: [{joined}]")
        hit = True
        i += 1
    if not hit:
        raise ValueError(f"key '{key}' not found")
    return "\n".join(out) + "\n"


def _yaml_params_of(root):
    if isinstance(root, dict):
        if '/**' in root and 'ros__parameters' in root['/**']:
            return root['/**']['ros__parameters']
        if 'ros__parameters' in root:
            return root['ros__parameters']
    return root


def _apply_edits_textually(path, slots):
    """Write the edited slots into the file by textual substitution, the
    same discipline as the daemon's holder-map persist: comments and
    untouched lines survive, the edited text is parsed back and the
    touched slots verified, and the replacement is tmp + atomic rename.
    Applied to the text on disk NOW, so a trim another writer landed
    since our load survives by construction."""
    with open(path) as f:
        text = f.read()
    for kind, key, index, value in slots:
        value = _round7(value)
        if kind == 'scalar':
            text = _set_scalar_line(text, key, value)
        else:
            text = _set_list_entry(text, key, index, value)
    tmp = path + '.new'
    with open(tmp, 'w') as f:
        f.write(text)
    try:
        with open(tmp) as f:
            params = _yaml_params_of(yaml.safe_load(f))
        for kind, key, index, value in slots:
            expected = _round7(value)
            got = params.get(key) if kind == 'scalar' else params.get(key, [])[index]
            if got != expected:
                raise ValueError(
                    f"verify failed for {key}[{index}]: wrote {expected!r}, "
                    f"read back {got!r}")
    except Exception:
        os.unlink(tmp)
        raise
    os.replace(tmp, path)


class OffsetTableWidget(qt.QWidget):
    """Table-based offset editor for all holders."""

    offset_changed = qt.Signal()  # Emitted when any offset changes

    def __init__(self, parent=None):
        super().__init__(parent)
        self._params = {}
        self._loaded_mm = {}  # (row, col) -> value loaded from YAML, to detect real edits
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # Create table
        self.table = qt.QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(
            ["Target", "X (mm)", "Y (mm)", "Z (mm)", "Tilt X (deg)", "Tilt Z (deg)"]
        )
        self.table.horizontalHeaderItem(2).setToolTip(
            "Tool y. For holders 2-10: per-holder insertion-depth trim,\n"
            "positive = deeper (holder_multi_y_offsets)."
        )
        self.table.horizontalHeaderItem(4).setToolTip(
            "Seat lean about tool x (deg). The Holder 1 row is the base\n"
            "applied to every holder (holder_on_position_tilt_x_deg);\n"
            "rows 2-10 add their own trim on top (holder_multi_tilt_x_deg)."
        )
        self.table.horizontalHeaderItem(5).setToolTip(
            "Seat lean about tool z / base y (deg). Same base + per-holder\n"
            "trim scheme (holder_on_position_tilt_z_deg /\n"
            "holder_multi_tilt_z_deg)."
        )
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

            # X, Y, Z (mm) and tilt x/z (deg) spinboxes
            for col in range(1, 6):
                spin = qt.QDoubleSpinBox()
                if col >= 4:
                    spin.setRange(-5.0, 5.0)
                    spin.setSingleStep(0.05)
                else:
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
                elif col == 3:  # Z
                    spin.setStyleSheet("QDoubleSpinBox { color: #88AAFF; background: #454545; border: 1px solid #555; }")
                elif col == 4:  # Tilt X
                    spin.setStyleSheet("QDoubleSpinBox { color: #FFCC66; background: #454545; border: 1px solid #555; }")
                else:  # Tilt Z
                    spin.setStyleSheet("QDoubleSpinBox { color: #FF99CC; background: #454545; border: 1px solid #555; }")

                # The stage seat has no tilt parameters
                if row == 0 and col >= 4:
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

        # Tilt: bases on the Holder 1 row, in degrees (no mm scaling)
        self._set_cell_value(1, 4, params.get('holder_on_position_tilt_x_deg', 0))
        self._set_cell_value(1, 5, params.get('holder_on_position_tilt_z_deg', 0))

        # Holder 2-10 (rows 2-10)
        x_offsets = params.get('holder_multi_x_offsets', [0] * 9)
        y_offsets = params.get('holder_multi_y_offsets', [0] * 9)
        z_offsets = params.get('holder_multi_z_offsets', [0] * 9)
        tilts = params.get('holder_multi_tilt_x_deg', [0] * 9)
        tilts_z = params.get('holder_multi_tilt_z_deg', [0] * 9)

        for i in range(9):
            row = i + 2
            if i < len(x_offsets):
                self._set_cell_value(row, 1, x_offsets[i] * 1000)
            if i < len(y_offsets):
                self._set_cell_value(row, 2, y_offsets[i] * 1000)
            if i < len(z_offsets):
                self._set_cell_value(row, 3, z_offsets[i] * 1000)
            if i < len(tilts):
                self._set_cell_value(row, 4, tilts[i])
            if i < len(tilts_z):
                self._set_cell_value(row, 5, tilts_z[i])

        self.table.blockSignals(False)

        # Snapshot the (quantized) value each cell was loaded with, so on save we
        # can tell which cells the operator actually edited and leave the rest of
        # the YAML untouched (e.g. don't rewrite Y when only X/Z were jogged).
        self._loaded_mm = {}
        for r in range(self.table.rowCount()):
            for c in (1, 2, 3, 4, 5):
                widget = self.table.cellWidget(r, c)
                if widget is not None:
                    self._loaded_mm[(r, c)] = widget.value()

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

    def _cell_edited(self, row, col):
        """True if the operator changed this cell since it was loaded.

        Comparing against the loaded (already 3-decimal-quantized) value means a
        round-trip with no edit reads as unchanged, so we never rewrite — and
        re-quantize — fields the operator never touched.
        """
        loaded = self._loaded_mm.get((row, col))
        return loaded is None or self._get_cell_value(row, col) != loaded

    def get_edited_slots(self):
        """The edited cells as write slots: ('scalar'|'list', key, index,
        file-unit value). The one mapping between table cells and YAML
        keys — both the merge view and the textual save consume it."""
        slots = []
        single_fields = [
            ('sample_holder_on_position_x_offset', 0, 1, 1e-3),
            ('sample_holder_on_position_y_offset', 0, 2, 1e-3),
            ('sample_holder_on_position_z_offset', 0, 3, 1e-3),
            ('holder1_on_position_x_offset', 1, 1, 1e-3),
            ('holder1_on_position_y_offset', 1, 2, 1e-3),
            ('holder1_on_position_z_offset', 1, 3, 1e-3),
            # Tilt bases are degrees, not mm: no scaling.
            ('holder_on_position_tilt_x_deg', 1, 4, 1.0),
            ('holder_on_position_tilt_z_deg', 1, 5, 1.0),
        ]
        for key, row, col, scale in single_fields:
            if self._cell_edited(row, col):
                slots.append(('scalar', key, None,
                              self._get_cell_value(row, col) * scale))
        list_fields = [
            ('holder_multi_x_offsets', 1, 1e-3),
            ('holder_multi_y_offsets', 2, 1e-3),
            ('holder_multi_z_offsets', 3, 1e-3),
            ('holder_multi_tilt_x_deg', 4, 1.0),
            ('holder_multi_tilt_z_deg', 5, 1.0),
        ]
        for i in range(9):
            row = i + 2
            for key, col, scale in list_fields:
                if self._cell_edited(row, col):
                    slots.append(('list', key, i,
                                  self._get_cell_value(row, col) * scale))
        return slots

    def get_updated_params(self, base=None):
        """Get updated parameters from table.

        Only fields whose cell was actually edited are overwritten; untouched
        fields keep their original full-precision YAML value. `base` (when
        given) supplies those untouched values instead of the load-time
        snapshot — a fresh read of the file lets a value someone else wrote
        since our load (the daemon's holder-map trim persist) survive.
        """
        params = dict(self._params if base is None else base)
        for kind, key, index, value in self.get_edited_slots():
            if kind == 'scalar':
                params[key] = value
            else:
                lst = list(params.get(key, [0.0] * 9))
                while len(lst) < 9:
                    lst.append(0.0)
                lst[index] = value
                params[key] = lst
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
        default_path = _repo_config_dir()
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

        # Textual edit of only the edited slots (comments survive, unlike
        # yaml.dump), applied to the text on disk NOW — the daemon's
        # holder map writes this file too, and a trim it landed since our
        # load must not be undone by this save.
        slots = self.offset_table.get_edited_slots()
        if not slots:
            qt.QMessageBox.information(
                self, "Saved", "No edited cells — file untouched.")
            return
        try:
            _apply_edits_textually(self._yaml_path, slots)
        except Exception as e:
            qt.QMessageBox.critical(self, "Error", f"Failed to save YAML:\n{e}")
            return

        # Re-seed the table from what was actually written, so cells that
        # picked up on-disk changes display them and the edit snapshot
        # resets.
        try:
            with open(self._yaml_path) as f:
                fresh = yaml.safe_load(f)
        except Exception as e:
            qt.QMessageBox.critical(
                self, "Error", f"Saved, but failed to re-read YAML:\n{e}")
            return
        fresh_params = _yaml_params_of(fresh)
        self._yaml_data = fresh
        self._params = fresh_params
        self.offset_table.set_params(fresh_params)
        self.path_label.setText(os.path.basename(self._yaml_path))
        self.path_label.setStyleSheet("color: #4CAF50; font-size: 9px;")
        qt.QMessageBox.information(self, "Saved", "YAML saved!\n다음 로봇 트리거 시 적용됩니다.")


def _repo_config_dir():
    """config/ of the checkout this GUI runs from."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "..", "config"))


class TcpJogWidget(qt.QGroupBox):
    """TCP Jog controls for calibration mode."""

    jog_requested = qt.Signal(str, int)  # axis ('x', 'y', 'z'), direction (-1 or +1)
    offset_changed = qt.Signal(float, float, float)  # x, y, z accumulated offset in mm

    def __init__(self, parent=None):
        super().__init__("TCP Jog (캘리브레이션 대기 중 사용)", parent)
        self._step_mm = 1.0
        # Base offset from YAML (set when calibration starts)
        self._base_x = 0.0
        self._base_y = 0.0
        self._base_z = 0.0
        # Accumulated jog offset
        self._jog_x = 0.0
        self._jog_y = 0.0
        self._jog_z = 0.0
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(6)

        # Accumulated offset display
        offset_group = qt.QWidget()
        offset_layout = qt.QGridLayout(offset_group)
        offset_layout.setContentsMargins(0, 0, 0, 0)
        offset_layout.setSpacing(4)

        # Headers
        offset_layout.addWidget(qt.QLabel(""), 0, 0)
        base_label = qt.QLabel("Base")
        base_label.setStyleSheet("color: #888; font-size: 9px;")
        base_label.setAlignment(qt.Qt.AlignCenter)
        offset_layout.addWidget(base_label, 0, 1)
        jog_label = qt.QLabel("Jog")
        jog_label.setStyleSheet("color: #888; font-size: 9px;")
        jog_label.setAlignment(qt.Qt.AlignCenter)
        offset_layout.addWidget(jog_label, 0, 2)
        total_label = qt.QLabel("Total")
        total_label.setStyleSheet("color: #fff; font-size: 9px; font-weight: bold;")
        total_label.setAlignment(qt.Qt.AlignCenter)
        offset_layout.addWidget(total_label, 0, 3)

        # X row
        x_axis = qt.QLabel("X:")
        x_axis.setStyleSheet("color: #ff6666; font-weight: bold;")
        offset_layout.addWidget(x_axis, 1, 0)
        self._x_base_label = qt.QLabel("0.000")
        self._x_base_label.setStyleSheet("color: #888; font-size: 10px;")
        self._x_base_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._x_base_label, 1, 1)
        self._x_jog_label = qt.QLabel("+0.000")
        self._x_jog_label.setStyleSheet("color: #ffaa66; font-size: 10px;")
        self._x_jog_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._x_jog_label, 1, 2)
        self._x_total_label = qt.QLabel("0.000")
        self._x_total_label.setStyleSheet("color: #ff6666; font-size: 11px; font-weight: bold;")
        self._x_total_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._x_total_label, 1, 3)

        # Y row
        y_axis = qt.QLabel("Y:")
        y_axis.setStyleSheet("color: #66ff66; font-weight: bold;")
        offset_layout.addWidget(y_axis, 2, 0)
        self._y_base_label = qt.QLabel("0.000")
        self._y_base_label.setStyleSheet("color: #888; font-size: 10px;")
        self._y_base_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._y_base_label, 2, 1)
        self._y_jog_label = qt.QLabel("+0.000")
        self._y_jog_label.setStyleSheet("color: #ffaa66; font-size: 10px;")
        self._y_jog_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._y_jog_label, 2, 2)
        self._y_total_label = qt.QLabel("0.000")
        self._y_total_label.setStyleSheet("color: #66ff66; font-size: 11px; font-weight: bold;")
        self._y_total_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._y_total_label, 2, 3)

        # Z row
        z_axis = qt.QLabel("Z:")
        z_axis.setStyleSheet("color: #6699ff; font-weight: bold;")
        offset_layout.addWidget(z_axis, 3, 0)
        self._z_base_label = qt.QLabel("0.000")
        self._z_base_label.setStyleSheet("color: #888; font-size: 10px;")
        self._z_base_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._z_base_label, 3, 1)
        self._z_jog_label = qt.QLabel("+0.000")
        self._z_jog_label.setStyleSheet("color: #ffaa66; font-size: 10px;")
        self._z_jog_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._z_jog_label, 3, 2)
        self._z_total_label = qt.QLabel("0.000")
        self._z_total_label.setStyleSheet("color: #6699ff; font-size: 11px; font-weight: bold;")
        self._z_total_label.setAlignment(qt.Qt.AlignRight)
        offset_layout.addWidget(self._z_total_label, 3, 3)

        layout.addWidget(offset_group)

        # Step size selector
        step_layout = qt.QHBoxLayout()
        step_layout.addWidget(qt.QLabel("Step:"))
        self.step_combo = qt.QComboBox()
        self.step_combo.addItem("0.1 mm", 0.1)
        self.step_combo.addItem("0.5 mm", 0.5)
        self.step_combo.addItem("1.0 mm", 1.0)
        self.step_combo.addItem("2.0 mm", 2.0)
        self.step_combo.addItem("5.0 mm", 5.0)
        self.step_combo.setCurrentIndex(2)  # Default 1.0mm
        self.step_combo.currentIndexChanged.connect(self._on_step_changed)
        step_layout.addWidget(self.step_combo)
        step_layout.addStretch()
        layout.addLayout(step_layout)

        # Jog buttons grid
        btn_layout = qt.QGridLayout()
        btn_layout.setSpacing(4)

        # X axis (Left/Right in TCP frame)
        x_minus = qt.QPushButton("X-")
        x_minus.setMinimumSize(55, 40)
        x_minus.setStyleSheet("""
            QPushButton { background-color: #d32f2f; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #b71c1c; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        x_minus.clicked.connect(lambda: self.jog_requested.emit('x', -1))
        btn_layout.addWidget(x_minus, 0, 0)

        x_label = qt.QLabel("X\n←왼쪽")
        x_label.setAlignment(qt.Qt.AlignCenter)
        x_label.setStyleSheet("color: #ff6666; font-size: 9px; font-weight: bold;")
        btn_layout.addWidget(x_label, 0, 1)

        x_plus = qt.QPushButton("X+")
        x_plus.setMinimumSize(55, 40)
        x_plus.setStyleSheet("""
            QPushButton { background-color: #d32f2f; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #b71c1c; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        x_plus.clicked.connect(lambda: self.jog_requested.emit('x', 1))
        btn_layout.addWidget(x_plus, 0, 2)

        # Y axis (Up/Down in TCP frame)
        y_minus = qt.QPushButton("Y-")
        y_minus.setMinimumSize(55, 40)
        y_minus.setStyleSheet("""
            QPushButton { background-color: #388e3c; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #2e7d32; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        y_minus.clicked.connect(lambda: self.jog_requested.emit('y', -1))
        btn_layout.addWidget(y_minus, 1, 0)

        y_label = qt.QLabel("Y\n↓아래")
        y_label.setAlignment(qt.Qt.AlignCenter)
        y_label.setStyleSheet("color: #66ff66; font-size: 9px; font-weight: bold;")
        btn_layout.addWidget(y_label, 1, 1)

        y_plus = qt.QPushButton("Y+")
        y_plus.setMinimumSize(55, 40)
        y_plus.setStyleSheet("""
            QPushButton { background-color: #388e3c; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #2e7d32; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        y_plus.clicked.connect(lambda: self.jog_requested.emit('y', 1))
        btn_layout.addWidget(y_plus, 1, 2)

        # Z axis (Forward/Backward in TCP frame)
        z_minus = qt.QPushButton("Z-")
        z_minus.setMinimumSize(55, 40)
        z_minus.setStyleSheet("""
            QPushButton { background-color: #1976d2; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        z_minus.clicked.connect(lambda: self.jog_requested.emit('z', -1))
        btn_layout.addWidget(z_minus, 2, 0)

        z_label = qt.QLabel("Z\n↑정면")
        z_label.setAlignment(qt.Qt.AlignCenter)
        z_label.setStyleSheet("color: #6699ff; font-size: 9px; font-weight: bold;")
        btn_layout.addWidget(z_label, 2, 1)

        z_plus = qt.QPushButton("Z+")
        z_plus.setMinimumSize(55, 40)
        z_plus.setStyleSheet("""
            QPushButton { background-color: #1976d2; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        z_plus.clicked.connect(lambda: self.jog_requested.emit('z', 1))
        btn_layout.addWidget(z_plus, 2, 2)

        layout.addLayout(btn_layout)

        # Apply to YAML button
        self.apply_btn = qt.QPushButton("Apply Total to YAML")
        self.apply_btn.setMinimumHeight(32)
        self.apply_btn.setStyleSheet("""
            QPushButton { background-color: #FF9800; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        layout.addWidget(self.apply_btn)

        # Store buttons for enable/disable
        self._jog_buttons = [x_minus, x_plus, y_minus, y_plus, z_minus, z_plus]

    def _on_step_changed(self, index):
        self._step_mm = self.step_combo.currentData()

    def get_step_mm(self):
        return self._step_mm

    def set_enabled(self, enabled):
        for btn in self._jog_buttons:
            btn.setEnabled(enabled)
        self.step_combo.setEnabled(enabled)
        self.apply_btn.setEnabled(enabled)

    def set_base_offset(self, x_mm, y_mm, z_mm):
        """Set base offset from YAML when calibration starts."""
        self._base_x = x_mm
        self._base_y = y_mm
        self._base_z = z_mm
        self._jog_x = 0.0
        self._jog_y = 0.0
        self._jog_z = 0.0
        self._update_display()

    def add_jog(self, axis, direction):
        """Add jog amount to accumulated offset."""
        delta = direction * self._step_mm
        if axis == 'x':
            self._jog_x += delta
        elif axis == 'y':
            self._jog_y += delta
        elif axis == 'z':
            self._jog_z += delta
        self._update_display()
        self.offset_changed.emit(
            self._base_x + self._jog_x,
            self._base_y + self._jog_y,
            self._base_z + self._jog_z
        )

    def get_total_offset(self):
        """Get total offset (base + jog) in mm."""
        return (
            self._base_x + self._jog_x,
            self._base_y + self._jog_y,
            self._base_z + self._jog_z
        )

    def get_jog_offset(self):
        """Get jog offset only (delta from base) in mm."""
        return (self._jog_x, self._jog_y, self._jog_z)

    def _update_display(self):
        """Update offset display labels."""
        # Base
        self._x_base_label.setText(f"{self._base_x:.3f}")
        self._y_base_label.setText(f"{self._base_y:.3f}")
        self._z_base_label.setText(f"{self._base_z:.3f}")
        # Jog (with sign)
        self._x_jog_label.setText(f"{self._jog_x:+.3f}")
        self._y_jog_label.setText(f"{self._jog_y:+.3f}")
        self._z_jog_label.setText(f"{self._jog_z:+.3f}")
        # Total
        self._x_total_label.setText(f"{self._base_x + self._jog_x:.3f}")
        self._y_total_label.setText(f"{self._base_y + self._jog_y:.3f}")
        self._z_total_label.setText(f"{self._base_z + self._jog_z:.3f}")

    def reset_jog(self):
        """Reset jog accumulation to zero."""
        self._jog_x = 0.0
        self._jog_y = 0.0
        self._jog_z = 0.0
        self._update_display()


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
        self._calib_mode = None  # 'holder' or 'sample'
        self._calib_holder_num = None
        self._setup_ui()
        self._connect_signals()
        taught = os.path.join(_repo_config_dir(), "taught_waypoints.yaml")
        if os.path.exists(taught):
            self.yaml_editor.load_yaml_file(taught)

    def _setup_ui(self):
        self.setWindowTitle("Calibration - TCP Coordinate & YAML Editor")
        self.setMinimumSize(950, 850)

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

        # TCP Jog controls
        self.jog_widget = TcpJogWidget()
        self.jog_widget.set_enabled(False)  # Initially disabled
        left_layout.addWidget(self.jog_widget)

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

        self.jog_widget.jog_requested.connect(self._on_jog_requested)
        self.jog_widget.apply_btn.clicked.connect(self._apply_jog_to_yaml)

    def _on_offset_changed(self, x_mm, y_mm, z_mm):
        """Update coordinate view when offset changes."""
        self.coord_view.set_offsets(x_mm, y_mm, z_mm)

    def _on_jog_requested(self, axis, direction):
        """Handle jog button press."""
        step_mm = self.jog_widget.get_step_mm()
        self.epics_handler.set_jog(axis, direction, step_mm)
        # Accumulate jog offset
        self.jog_widget.add_jog(axis, direction)
        # Update coordinate view with total offset
        x, y, z = self.jog_widget.get_total_offset()
        self.coord_view.set_offsets(x, y, z)
        # Update status
        jog_x, jog_y, jog_z = self.jog_widget.get_jog_offset()
        self.status_label.setText(
            f"Jog: {axis.upper()} {'+' if direction > 0 else '-'}{step_mm}mm\n\n"
            f"누적 Jog: X={jog_x:+.2f} Y={jog_y:+.2f} Z={jog_z:+.2f}\n"
            f"Total: X={x:.3f} Y={y:.3f} Z={z:.3f}"
        )
        self.status_label.setStyleSheet("color: #2196F3;")

    def _start_holder_calib(self, holder_num):
        self.epics_handler.set_holder(holder_num)
        self.epics_handler.set_calib_mode(1)
        self.epics_handler.trigger_sequence()

        self._calibrating = True
        self._calib_mode = 'holder'
        self._calib_holder_num = holder_num
        self.calib_controls.set_calibrating(True)
        self.jog_widget.set_enabled(True)

        # Get current holder offset from YAML as base
        x, y, z = self._get_holder_offset_from_yaml(holder_num)
        self.jog_widget.set_base_offset(x, y, z)
        self.coord_view.set_offsets(x, y, z)

        self.status_label.setText(
            f"Holder {holder_num} Calibration\n\n"
            "로봇 이동 중...\n\n"
            "Jog 버튼으로 TCP 위치 조정 가능\n"
            "→ Save YAML → Continue"
        )
        self.status_label.setStyleSheet("color: #9C27B0;")

    def _start_sample_calib(self):
        holder_num = self.calib_controls.holder_combo.currentData()
        self.epics_handler.set_holder(holder_num)
        self.epics_handler.set_calib_mode(2)
        self.epics_handler.trigger_sequence()

        self._calibrating = True
        self._calib_mode = 'sample'
        self.calib_controls.set_calibrating(True)
        self.jog_widget.set_enabled(True)

        # Get current sample holder offset from YAML as base
        x, y, z = self._get_sample_holder_offset_from_yaml()
        self.jog_widget.set_base_offset(x, y, z)
        self.coord_view.set_offsets(x, y, z)

        self.status_label.setText(
            f"Sample Holder Calibration\n(from Holder {holder_num})\n\n"
            "로봇 이동 중...\n\n"
            "Jog 버튼으로 TCP 위치 조정 가능\n"
            "→ Save YAML → Continue"
        )
        self.status_label.setStyleSheet("color: #673AB7;")

    def _continue_calib(self):
        self.epics_handler.trigger_sequence()
        self._calibrating = False
        self.calib_controls.set_calibrating(False)
        self.jog_widget.set_enabled(False)
        self.status_label.setText("샘플 복귀 중...")
        self.status_label.setStyleSheet("color: #4CAF50;")

    def _abort_calib(self):
        self.epics_handler.set_wait(2)
        self.epics_handler.trigger_sequence()
        self._calibrating = False
        self.calib_controls.set_calibrating(False)
        self.jog_widget.set_enabled(False)
        self.status_label.setText("캘리브레이션 중단\n샘플 복귀 중...")
        self.status_label.setStyleSheet("color: #f44336;")

    def _apply_jog_to_yaml(self):
        """Apply total jog offset to YAML table."""
        x, y, z = self.jog_widget.get_total_offset()

        if self._calib_mode == 'holder':
            holder_num = self._calib_holder_num
            if holder_num == 1:
                # Holder 1: X, Y, Z all (row 1)
                self.yaml_editor.offset_table._set_cell_value(1, 1, x)
                self.yaml_editor.offset_table._set_cell_value(1, 2, y)
                self.yaml_editor.offset_table._set_cell_value(1, 3, z)
                self.yaml_editor._mark_modified()
                self.status_label.setText(
                    f"Holder 1 offset 적용됨:\n"
                    f"X={x:.3f} Y={y:.3f} Z={z:.3f}\n\n"
                    "Save YAML 버튼으로 저장하세요"
                )
            else:
                # Holder 2~10: X, Z only (row = holder_num, Y is based on Holder 1)
                row = holder_num  # row 2 = Holder 2, etc.
                self.yaml_editor.offset_table._set_cell_value(row, 1, x)
                self.yaml_editor.offset_table._set_cell_value(row, 3, z)
                self.yaml_editor._mark_modified()
                self.status_label.setText(
                    f"Holder {holder_num} offset 적용됨:\n"
                    f"X={x:.3f} Z={z:.3f}\n"
                    "(Y는 Holder 1 기준)\n\n"
                    "Save YAML 버튼으로 저장하세요"
                )
        elif self._calib_mode == 'sample':
            # Sample Holder: X, Y, Z all (row 0)
            self.yaml_editor.offset_table._set_cell_value(0, 1, x)
            self.yaml_editor.offset_table._set_cell_value(0, 2, y)
            self.yaml_editor.offset_table._set_cell_value(0, 3, z)
            self.yaml_editor._mark_modified()
            self.status_label.setText(
                f"Sample Holder offset 적용됨:\n"
                f"X={x:.3f} Y={y:.3f} Z={z:.3f}\n\n"
                "Save YAML 버튼으로 저장하세요"
            )

        self.status_label.setStyleSheet("color: #FF9800;")

    def _get_holder_offset_from_yaml(self, holder_num=1):
        """Get holder offset from loaded YAML (in mm)."""
        params = self.yaml_editor._params
        if holder_num == 1:
            x = params.get('holder1_on_position_x_offset', 0) * 1000
            y = params.get('holder1_on_position_y_offset', 0) * 1000
            z = params.get('holder1_on_position_z_offset', 0) * 1000
        else:
            # Holder 2~10: base from holder1 + multi offsets
            x_offsets = params.get('holder_multi_x_offsets', [0] * 9)
            z_offsets = params.get('holder_multi_z_offsets', [0] * 9)
            idx = holder_num - 2  # holder 2 -> index 0
            x = x_offsets[idx] * 1000 if idx < len(x_offsets) else 0
            y = params.get('holder1_on_position_y_offset', 0) * 1000  # Y from holder1
            z = z_offsets[idx] * 1000 if idx < len(z_offsets) else 0
        return (x, y, z)

    def _get_sample_holder_offset_from_yaml(self):
        """Get sample holder offset from loaded YAML (in mm)."""
        params = self.yaml_editor._params
        x = params.get('sample_holder_on_position_x_offset', 0) * 1000
        y = params.get('sample_holder_on_position_y_offset', 0) * 1000
        z = params.get('sample_holder_on_position_z_offset', 0) * 1000
        return (x, y, z)

    def load_yaml(self, path):
        """Load YAML file programmatically."""
        self.yaml_editor.load_yaml_file(path)
