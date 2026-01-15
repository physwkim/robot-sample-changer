"""Main Control Panel Widget using silx.gui."""

from silx.gui import qt
from silx.gui.widgets.WaitingPushButton import WaitingPushButton


class HolderSelector(qt.QGroupBox):
    """Widget for selecting holder number."""

    holder_changed = qt.Signal(int)

    def __init__(self, parent=None):
        super().__init__("Holder Selection", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QGridLayout(self)
        layout.setSpacing(8)

        self.buttons = []
        for i in range(10):
            btn = qt.QPushButton(f"{i + 1}")
            btn.setCheckable(True)
            btn.setMinimumSize(50, 50)
            font = qt.QFont()
            font.setPointSize(14)
            font.setBold(True)
            btn.setFont(font)
            btn.clicked.connect(lambda checked, num=i+1: self._on_holder_clicked(num))
            row, col = divmod(i, 5)
            layout.addWidget(btn, row, col)
            self.buttons.append(btn)

        self.buttons[0].setChecked(True)
        self._current_holder = 1

    def _on_holder_clicked(self, holder_num):
        for i, btn in enumerate(self.buttons):
            btn.setChecked(i + 1 == holder_num)
        self._current_holder = holder_num
        self.holder_changed.emit(holder_num)

    def get_selected_holder(self):
        return self._current_holder

    def set_selected_holder(self, holder_num):
        if 1 <= holder_num <= 10:
            self._on_holder_clicked(holder_num)


class SampleOperations(qt.QGroupBox):
    """Widget for sample holder operations."""

    put_to_sample_clicked = qt.Signal()
    remove_from_sample_clicked = qt.Signal()
    continue_clicked = qt.Signal()
    abort_clicked = qt.Signal()

    def __init__(self, parent=None):
        super().__init__("Sample Holder Operations", parent)
        self._is_waiting = False
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(12)

        font = qt.QFont()
        font.setPointSize(12)

        # Put to sample holder
        self.put_btn = WaitingPushButton("Put to Sample Holder")
        self.put_btn.setMinimumHeight(60)
        self.put_btn.setFont(font)
        self.put_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 8px;
                padding: 10px;
            }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:pressed { background-color: #3d8b40; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.put_btn.clicked.connect(self.put_to_sample_clicked.emit)
        layout.addWidget(self.put_btn)

        # Remove from sample holder
        self.remove_btn = WaitingPushButton("Remove from Sample Holder")
        self.remove_btn.setMinimumHeight(60)
        self.remove_btn.setFont(font)
        self.remove_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border-radius: 8px;
                padding: 10px;
            }
            QPushButton:hover { background-color: #1976D2; }
            QPushButton:pressed { background-color: #1565C0; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.remove_btn.clicked.connect(self.remove_from_sample_clicked.emit)
        layout.addWidget(self.remove_btn)

        # Separator
        line = qt.QFrame()
        line.setFrameShape(qt.QFrame.HLine)
        line.setFrameShadow(qt.QFrame.Sunken)
        layout.addWidget(line)

        # Wait control buttons (Continue / Abort)
        wait_layout = qt.QHBoxLayout()

        self.continue_btn = qt.QPushButton("Continue")
        self.continue_btn.setMinimumHeight(50)
        self.continue_btn.setFont(font)
        self.continue_btn.setStyleSheet("""
            QPushButton {
                background-color: #8BC34A;
                color: white;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #7CB342; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.continue_btn.setEnabled(False)
        self.continue_btn.clicked.connect(self.continue_clicked.emit)
        wait_layout.addWidget(self.continue_btn)

        self.abort_btn = qt.QPushButton("Abort")
        self.abort_btn.setMinimumHeight(50)
        self.abort_btn.setFont(font)
        self.abort_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF5722;
                color: white;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #E64A19; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.abort_btn.setEnabled(False)
        self.abort_btn.clicked.connect(self.abort_clicked.emit)
        wait_layout.addWidget(self.abort_btn)

        layout.addLayout(wait_layout)

    def set_waiting_for_measurement(self, waiting):
        """Enable Continue/Abort when waiting for measurement."""
        self._is_waiting = waiting
        self.continue_btn.setEnabled(waiting)
        self.abort_btn.setEnabled(waiting)
        # Put is disabled while waiting, but Remove stays enabled
        self.put_btn.setEnabled(not waiting)

    def set_running(self, running):
        """Set running state (sequence in progress)."""
        self.put_btn.setWaiting(running)

    def set_enabled(self, enabled):
        self.put_btn.setEnabled(enabled)
        self.remove_btn.setEnabled(enabled)


class StatusDisplay(qt.QGroupBox):
    """Widget for displaying robot status."""

    def __init__(self, parent=None):
        super().__init__("Status", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QGridLayout(self)
        layout.setSpacing(8)

        bold_font = qt.QFont()
        bold_font.setPointSize(14)
        bold_font.setBold(True)

        # Connection status
        layout.addWidget(qt.QLabel("Connection:"), 0, 0)
        self.connection_label = qt.QLabel("Disconnected")
        self.connection_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.connection_label, 0, 1)

        # Current step
        layout.addWidget(qt.QLabel("Current Step:"), 1, 0)
        self.step_label = qt.QLabel("0")
        self.step_label.setFont(bold_font)
        layout.addWidget(self.step_label, 1, 1)

        # Holder
        layout.addWidget(qt.QLabel("Active Holder:"), 2, 0)
        self.holder_label = qt.QLabel("1")
        self.holder_label.setFont(bold_font)
        layout.addWidget(self.holder_label, 2, 1)

        # Gripper status
        layout.addWidget(qt.QLabel("Gripper:"), 3, 0)
        self.gripper_label = qt.QLabel("Unknown")
        layout.addWidget(self.gripper_label, 3, 1)

        # Calibration mode
        layout.addWidget(qt.QLabel("Mode:"), 4, 0)
        self.mode_label = qt.QLabel("Normal")
        layout.addWidget(self.mode_label, 4, 1)

        # Sample loaded status
        layout.addWidget(qt.QLabel("Sample:"), 5, 0)
        self.loaded_label = qt.QLabel("Not Loaded")
        self.loaded_label.setStyleSheet("color: gray; font-weight: bold;")
        layout.addWidget(self.loaded_label, 5, 1)

    def set_connected(self, connected):
        if connected:
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet("color: red; font-weight: bold;")

    def set_current_step(self, step):
        self.step_label.setText(str(step))

    def set_holder(self, holder):
        self.holder_label.setText(str(holder))

    def set_gripper_state(self, is_open):
        if is_open:
            self.gripper_label.setText("Open")
            self.gripper_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.gripper_label.setText("Closed")
            self.gripper_label.setStyleSheet("color: orange; font-weight: bold;")

    def set_mode(self, mode):
        modes = ["Normal", "Holder Calib", "Sample Calib"]
        self.mode_label.setText(modes[mode] if 0 <= mode < len(modes) else "Unknown")

    def set_loaded(self, loaded):
        if loaded:
            self.loaded_label.setText("Loaded")
            self.loaded_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.loaded_label.setText("Not Loaded")
            self.loaded_label.setStyleSheet("color: gray; font-weight: bold;")


class ControlPanel(qt.QWidget):
    """Main control panel combining all control widgets."""

    put_to_sample_requested = qt.Signal(int)
    remove_from_sample_requested = qt.Signal(int)
    continue_requested = qt.Signal()
    abort_requested = qt.Signal()
    open_calibration_requested = qt.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(16)

        self.holder_selector = HolderSelector()
        layout.addWidget(self.holder_selector)

        self.sample_ops = SampleOperations()
        layout.addWidget(self.sample_ops)

        self.status_display = StatusDisplay()
        layout.addWidget(self.status_display)

        self.calib_btn = qt.QPushButton("Open Calibration Window")
        self.calib_btn.setMinimumHeight(45)
        self.calib_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border-radius: 8px;
                padding: 10px;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #F57C00; }
        """)
        self.calib_btn.clicked.connect(self.open_calibration_requested.emit)
        layout.addWidget(self.calib_btn)

        layout.addStretch()

    def _connect_signals(self):
        self.sample_ops.put_to_sample_clicked.connect(
            lambda: self.put_to_sample_requested.emit(
                self.holder_selector.get_selected_holder()
            )
        )
        self.sample_ops.remove_from_sample_clicked.connect(
            lambda: self.remove_from_sample_requested.emit(
                self.holder_selector.get_selected_holder()
            )
        )
        self.sample_ops.continue_clicked.connect(self.continue_requested.emit)
        self.sample_ops.abort_clicked.connect(self.abort_requested.emit)
