"""Main Control Panel Widget using silx.gui."""

from silx.gui import qt


# Step number -> what the arm is doing there. Shown next to CurrentStep
# so the operator never has to memorize the step map.
STEP_NAMES = {
    0: "open gripper",
    1: "holder standby",
    2: "above holder",
    3: "at holder seat",
    4: "grip puck",
    5: "lift",
    6: "retreat",
    7: "stage standby",
    8: "above stage",
    9: "on stage seat",
    10: "release",
    11: "lift",
    12: "stage standby — waiting",
    13: "above stage",
    14: "on stage seat",
    15: "grip puck",
    16: "lift",
    17: "stage standby",
    18: "holder standby",
    19: "above holder",
    20: "at holder seat",
    21: "release",
    22: "lift",
    23: "holder standby",
}

# Robot:CalibMode enum, index == PV value.
MODE_NAMES = [
    "Normal",
    "Holder Calib",
    "Sample Holder Calib",
    "Hand-Eye Calib",
    "Recover",
    "Seat Probe",
    "Holder Map",
]

_GREEN = """
    QPushButton {
        background-color: #4CAF50; color: white;
        border-radius: 8px; padding: 10px;
    }
    QPushButton:hover { background-color: #45a049; }
    QPushButton:pressed { background-color: #3d8b40; }
    QPushButton:disabled { background-color: #cccccc; }
"""
_BLUE = """
    QPushButton {
        background-color: #2196F3; color: white;
        border-radius: 8px; padding: 10px;
    }
    QPushButton:hover { background-color: #1976D2; }
    QPushButton:pressed { background-color: #1565C0; }
    QPushButton:disabled { background-color: #cccccc; }
"""
_LIGHT_GREEN = """
    QPushButton {
        background-color: #8BC34A; color: white; border-radius: 8px;
    }
    QPushButton:hover { background-color: #7CB342; }
    QPushButton:disabled { background-color: #cccccc; }
"""
_RED = """
    QPushButton {
        background-color: #FF5722; color: white; border-radius: 8px;
    }
    QPushButton:hover { background-color: #E64A19; }
    QPushButton:disabled { background-color: #cccccc; }
"""
_AMBER = """
    QPushButton {
        background-color: #FF9800; color: white;
        border-radius: 8px; padding: 8px;
    }
    QPushButton:hover { background-color: #F57C00; }
    QPushButton:checked { background-color: #E65100; }
    QPushButton:disabled { background-color: #cccccc; }
"""
_GRAY = """
    QPushButton {
        background-color: #607D8B; color: white;
        border-radius: 8px; padding: 8px;
    }
    QPushButton:hover { background-color: #546E7A; }
    QPushButton:disabled { background-color: #cccccc; }
"""


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
            btn.clicked.connect(lambda checked, num=i + 1: self._on_holder_clicked(num))
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
    """Mount/return operations plus the run controls that pair with them."""

    mount_clicked = qt.Signal()
    return_clicked = qt.Signal()
    continue_clicked = qt.Signal()
    abort_clicked = qt.Signal()
    pause_toggled = qt.Signal(bool)
    recover_clicked = qt.Signal()

    def __init__(self, parent=None):
        super().__init__("Sample Operations", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(10)

        font = qt.QFont()
        font.setPointSize(12)

        self.mount_btn = qt.QPushButton("Mount Holder 1 → Stage")
        self.mount_btn.setMinimumHeight(56)
        self.mount_btn.setFont(font)
        self.mount_btn.setStyleSheet(_GREEN)
        self.mount_btn.clicked.connect(self.mount_clicked.emit)
        layout.addWidget(self.mount_btn)

        self.return_btn = qt.QPushButton("Return Stage → Holder 1")
        self.return_btn.setMinimumHeight(56)
        self.return_btn.setFont(font)
        self.return_btn.setStyleSheet(_BLUE)
        self.return_btn.clicked.connect(self.return_clicked.emit)
        layout.addWidget(self.return_btn)

        line = qt.QFrame()
        line.setFrameShape(qt.QFrame.HLine)
        line.setFrameShadow(qt.QFrame.Sunken)
        layout.addWidget(line)

        # At the stage-standby wait (step 12): Continue retrieves the
        # sample, Abort leaves it on the stage and stops.
        wait_layout = qt.QHBoxLayout()

        self.continue_btn = qt.QPushButton("Continue")
        self.continue_btn.setMinimumHeight(48)
        self.continue_btn.setFont(font)
        self.continue_btn.setStyleSheet(_LIGHT_GREEN)
        self.continue_btn.setEnabled(False)
        self.continue_btn.setToolTip(
            "At the stage-standby wait: retrieve the sample back to the holder"
        )
        self.continue_btn.clicked.connect(self.continue_clicked.emit)
        wait_layout.addWidget(self.continue_btn)

        self.abort_btn = qt.QPushButton("Abort")
        self.abort_btn.setMinimumHeight(48)
        self.abort_btn.setFont(font)
        self.abort_btn.setStyleSheet(_RED)
        self.abort_btn.setEnabled(False)
        self.abort_btn.setToolTip(
            "At the stage-standby wait: leave the sample on the stage and stop"
        )
        self.abort_btn.clicked.connect(self.abort_clicked.emit)
        wait_layout.addWidget(self.abort_btn)

        layout.addLayout(wait_layout)

        run_layout = qt.QHBoxLayout()

        self.pause_btn = qt.QPushButton("Pause after current step")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setMinimumHeight(42)
        self.pause_btn.setStyleSheet(_AMBER)
        self.pause_btn.toggled.connect(self.pause_toggled.emit)
        run_layout.addWidget(self.pause_btn)

        self.recover_btn = qt.QPushButton("Recover to Standby")
        self.recover_btn.setMinimumHeight(42)
        self.recover_btn.setStyleSheet(_GRAY)
        self.recover_btn.setToolTip(
            "After a protective stop or an aborted run: unlock, resend the\n"
            "robot program, and walk the arm back to holder standby.\n"
            "The gripper is not touched."
        )
        self.recover_btn.clicked.connect(self.recover_clicked.emit)
        run_layout.addWidget(self.recover_btn)

        layout.addLayout(run_layout)

    def set_holder(self, holder_num):
        self.mount_btn.setText(f"Mount Holder {holder_num} → Stage")
        self.return_btn.setText(f"Return Stage → Holder {holder_num}")

    def set_waiting(self, waiting):
        """Swap emphasis to Continue/Abort during the stage-standby wait."""
        self.continue_btn.setEnabled(waiting)
        self.abort_btn.setEnabled(waiting)

    def set_paused(self, paused):
        self.pause_btn.blockSignals(True)
        self.pause_btn.setChecked(paused)
        self.pause_btn.setText(
            "Paused — click to resume" if paused else "Pause after current step"
        )
        self.pause_btn.blockSignals(False)

    def set_enabled(self, enabled):
        self.mount_btn.setEnabled(enabled)
        self.return_btn.setEnabled(enabled)
        self.pause_btn.setEnabled(enabled)
        self.recover_btn.setEnabled(enabled)


class HolderMapGroup(qt.QGroupBox):
    """One-trigger seat probe of a holder (CalibMode=6)."""

    map_clicked = qt.Signal()

    def __init__(self, parent=None):
        super().__init__("Holder Map (seat probe)", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QHBoxLayout(self)
        layout.setSpacing(8)

        layout.addWidget(qt.QLabel("Puck from:"))
        self.source_combo = qt.QComboBox()
        self.source_combo.addItem("target holder itself", 0)
        for i in range(1, 11):
            self.source_combo.addItem(f"holder {i}", i)
        self.source_combo.setToolTip(
            "Where to fetch the puck that probes the seat.\n"
            "'target holder itself' probes with the puck already there."
        )
        layout.addWidget(self.source_combo, 1)

        self.map_btn = qt.QPushButton("Map Holder 1")
        self.map_btn.setMinimumHeight(42)
        self.map_btn.setStyleSheet(_GRAY)
        self.map_btn.setToolTip(
            "Seat the puck in the selected holder via the stage, probe the\n"
            "seat, leave the puck there, and return to standby. One trigger."
        )
        self.map_btn.clicked.connect(self.map_clicked.emit)
        layout.addWidget(self.map_btn)

    def get_source(self):
        return self.source_combo.currentData()

    def set_holder(self, holder_num):
        self.map_btn.setText(f"Map Holder {holder_num}")

    def set_enabled(self, enabled):
        self.map_btn.setEnabled(enabled)


class AdvancedGroup(qt.QGroupBox):
    """Any mode, any start step: the raw trigger, for resume and the
    two-trigger calibration modes."""

    trigger_clicked = qt.Signal()
    pause_step_changed = qt.Signal(int)

    def __init__(self, parent=None):
        super().__init__("Advanced / Resume", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QGridLayout(self)
        layout.setSpacing(8)

        layout.addWidget(qt.QLabel("Mode:"), 0, 0)
        self.mode_combo = qt.QComboBox()
        for i, name in enumerate(MODE_NAMES):
            self.mode_combo.addItem(f"{i} — {name}", i)
        layout.addWidget(self.mode_combo, 0, 1)

        self.trigger_btn = qt.QPushButton("Trigger")
        self.trigger_btn.setMinimumHeight(38)
        self.trigger_btn.setStyleSheet(_AMBER)
        layout.addWidget(self.trigger_btn, 0, 2)
        self.trigger_btn.clicked.connect(self.trigger_clicked.emit)

        layout.addWidget(qt.QLabel("StartStep:"), 1, 0)
        self.start_spin = qt.QSpinBox()
        self.start_spin.setRange(0, 23)
        self.start_spin.setToolTip(
            "Steps below this are skipped. After a crash or abort, set the\n"
            "interrupted step shown in Status and trigger to resume."
        )
        layout.addWidget(self.start_spin, 1, 1)

        pause_layout = qt.QHBoxLayout()
        pause_layout.addWidget(qt.QLabel("PauseStep:"))
        self.pause_spin = qt.QSpinBox()
        self.pause_spin.setRange(0, 23)
        self.pause_spin.setSpecialValueText("off")
        self.pause_spin.setToolTip("Pause when the sequence reaches this step (0 = off)")
        self.pause_spin.valueChanged.connect(self.pause_step_changed.emit)
        pause_layout.addWidget(self.pause_spin)
        layout.addLayout(pause_layout, 1, 2)

    def get_mode(self):
        return self.mode_combo.currentData()

    def get_start_step(self):
        return self.start_spin.value()

    def set_pause_step(self, step):
        self.pause_spin.blockSignals(True)
        self.pause_spin.setValue(step)
        self.pause_spin.blockSignals(False)

    def set_enabled(self, enabled):
        self.trigger_btn.setEnabled(enabled)


class StatusDisplay(qt.QGroupBox):
    """Widget for displaying robot status."""

    def __init__(self, parent=None):
        super().__init__("Status", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QGridLayout(self)
        layout.setSpacing(8)

        bold_font = qt.QFont()
        bold_font.setPointSize(13)
        bold_font.setBold(True)

        layout.addWidget(qt.QLabel("Connection:"), 0, 0)
        self.connection_label = qt.QLabel("Disconnected")
        self.connection_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.connection_label, 0, 1)

        layout.addWidget(qt.QLabel("Mode:"), 0, 2)
        self.mode_label = qt.QLabel("Normal")
        self.mode_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.mode_label, 0, 3)

        layout.addWidget(qt.QLabel("Step:"), 1, 0)
        self.step_label = qt.QLabel("0")
        self.step_label.setFont(bold_font)
        layout.addWidget(self.step_label, 1, 1, 1, 3)

        layout.addWidget(qt.QLabel("Holder:"), 2, 0)
        self.holder_label = qt.QLabel("1")
        self.holder_label.setFont(bold_font)
        layout.addWidget(self.holder_label, 2, 1)

        layout.addWidget(qt.QLabel("Gripper:"), 2, 2)
        self.gripper_label = qt.QLabel("Unknown")
        layout.addWidget(self.gripper_label, 2, 3)

        layout.addWidget(qt.QLabel("Sample:"), 3, 0)
        self.loaded_label = qt.QLabel("Not Loaded")
        self.loaded_label.setStyleSheet("color: gray; font-weight: bold;")
        layout.addWidget(self.loaded_label, 3, 1)

        layout.addWidget(qt.QLabel("Motion:"), 3, 2)
        self.motion_label = qt.QLabel("Run")
        layout.addWidget(self.motion_label, 3, 3)

    def set_connected(self, connected):
        if connected:
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet("color: red; font-weight: bold;")

    def set_current_step(self, step):
        name = STEP_NAMES.get(step)
        if step == 0:
            self.step_label.setText("0 — idle")
            self.step_label.setStyleSheet("")
        else:
            text = f"{step} — {name}" if name else str(step)
            self.step_label.setText(text)
            self.step_label.setStyleSheet("color: #1565C0;")

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
        if 0 <= mode < len(MODE_NAMES):
            self.mode_label.setText(MODE_NAMES[mode])
        else:
            self.mode_label.setText(str(mode))

    def set_loaded(self, loaded):
        if loaded:
            self.loaded_label.setText("Loaded")
            self.loaded_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.loaded_label.setText("Not Loaded")
            self.loaded_label.setStyleSheet("color: gray; font-weight: bold;")

    def set_motion(self, paused):
        if paused:
            self.motion_label.setText("Pause requested")
            self.motion_label.setStyleSheet("color: #E65100; font-weight: bold;")
        else:
            self.motion_label.setText("Run")
            self.motion_label.setStyleSheet("")


class ControlPanel(qt.QWidget):
    """Main control panel combining all control widgets."""

    mount_requested = qt.Signal(int)
    return_requested = qt.Signal(int)
    map_requested = qt.Signal(int, int)  # holder, source
    continue_requested = qt.Signal()
    abort_requested = qt.Signal()
    pause_toggled = qt.Signal(bool)
    recover_requested = qt.Signal()
    trigger_requested = qt.Signal(int, int)  # mode, start_step
    pause_step_changed = qt.Signal(int)
    open_calibration_requested = qt.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        layout = qt.QVBoxLayout(self)
        layout.setSpacing(14)

        self.status_display = StatusDisplay()
        layout.addWidget(self.status_display)

        self.holder_selector = HolderSelector()
        layout.addWidget(self.holder_selector)

        self.sample_ops = SampleOperations()
        layout.addWidget(self.sample_ops)

        self.holder_map = HolderMapGroup()
        layout.addWidget(self.holder_map)

        self.advanced = AdvancedGroup()
        layout.addWidget(self.advanced)

        self.calib_btn = qt.QPushButton("Open Calibration Window")
        self.calib_btn.setMinimumHeight(40)
        self.calib_btn.setStyleSheet(_AMBER)
        self.calib_btn.clicked.connect(self.open_calibration_requested.emit)
        layout.addWidget(self.calib_btn)

        layout.addStretch()

    def _connect_signals(self):
        holder = self.holder_selector.get_selected_holder
        self.sample_ops.mount_clicked.connect(
            lambda: self.mount_requested.emit(holder())
        )
        self.sample_ops.return_clicked.connect(
            lambda: self.return_requested.emit(holder())
        )
        self.holder_map.map_clicked.connect(
            lambda: self.map_requested.emit(holder(), self.holder_map.get_source())
        )
        self.sample_ops.continue_clicked.connect(self.continue_requested.emit)
        self.sample_ops.abort_clicked.connect(self.abort_requested.emit)
        self.sample_ops.pause_toggled.connect(self.pause_toggled.emit)
        self.sample_ops.recover_clicked.connect(self.recover_requested.emit)
        self.advanced.trigger_clicked.connect(
            lambda: self.trigger_requested.emit(
                self.advanced.get_mode(), self.advanced.get_start_step()
            )
        )
        self.advanced.pause_step_changed.connect(self.pause_step_changed.emit)

        self.holder_selector.holder_changed.connect(self.sample_ops.set_holder)
        self.holder_selector.holder_changed.connect(self.holder_map.set_holder)
