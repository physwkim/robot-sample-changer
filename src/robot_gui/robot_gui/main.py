"""Main application using silx.gui."""

import sys
from silx.gui import qt

from .epics_handler import EpicsHandler
from .control_panel import ControlPanel
from .gripper_widget import GripperWidget
from .calibration_window import CalibrationWindow


class RobotControlMainWindow(qt.QMainWindow):
    """Main window for robot control GUI."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR3e + HandE Robot Control")
        self.setMinimumSize(600, 750)

        self.epics_handler = EpicsHandler(self)
        self._calibration_window = None
        self._waiting_for_measurement = False

        self._setup_ui()
        self._setup_menu()
        self._connect_signals()
        self._connect_epics()

    def _setup_ui(self):
        central_widget = qt.QWidget()
        self.setCentralWidget(central_widget)

        layout = qt.QVBoxLayout(central_widget)
        layout.setSpacing(16)
        layout.setContentsMargins(16, 16, 16, 16)

        self.control_panel = ControlPanel()
        layout.addWidget(self.control_panel)

        self.gripper_widget = GripperWidget()
        layout.addWidget(self.gripper_widget)

        self.emergency_btn = self._create_emergency_button()
        layout.addWidget(self.emergency_btn)

        self.status_bar = qt.QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Connecting to EPICS...")

    def _create_emergency_button(self):
        btn = qt.QPushButton("STOP")
        btn.setMinimumHeight(60)
        font = qt.QFont()
        font.setPointSize(18)
        font.setBold(True)
        btn.setFont(font)
        btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                border: 3px solid #b71c1c;
                border-radius: 10px;
            }
            QPushButton:hover { background-color: #c62828; }
            QPushButton:pressed { background-color: #b71c1c; }
        """)
        btn.clicked.connect(self._on_emergency_stop)
        return btn

    def _setup_menu(self):
        menubar = self.menuBar()

        file_menu = menubar.addMenu("File")

        reconnect_action = qt.QAction("Reconnect EPICS", self)
        reconnect_action.triggered.connect(self._connect_epics)
        file_menu.addAction(reconnect_action)

        file_menu.addSeparator()

        exit_action = qt.QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        tools_menu = menubar.addMenu("Tools")

        calib_action = qt.QAction("Calibration Window", self)
        calib_action.triggered.connect(self._open_calibration)
        tools_menu.addAction(calib_action)

    def _connect_signals(self):
        # Control panel
        self.control_panel.put_to_sample_requested.connect(self._put_to_sample)
        self.control_panel.remove_from_sample_requested.connect(self._remove_from_sample)
        self.control_panel.continue_requested.connect(self._continue_sequence)
        self.control_panel.abort_requested.connect(self._abort_sequence)
        self.control_panel.open_calibration_requested.connect(self._open_calibration)
        self.control_panel.holder_selector.holder_changed.connect(
            self.epics_handler.set_holder
        )

        # Gripper
        self.gripper_widget.open_requested.connect(self.epics_handler.open_gripper)
        self.gripper_widget.close_requested.connect(self.epics_handler.close_gripper)

        # EPICS
        self.epics_handler.connection_changed.connect(self._on_connection_changed)
        self.epics_handler.current_step_changed.connect(self._on_step_changed)
        self.epics_handler.gripper_rbv_changed.connect(self._on_gripper_changed)
        self.epics_handler.holder_changed.connect(self._on_holder_changed)
        self.epics_handler.calib_mode_changed.connect(self._on_mode_changed)
        self.epics_handler.wait_changed.connect(self._on_wait_changed)

    def _connect_epics(self):
        self.status_bar.showMessage("Connecting to EPICS...")
        if self.epics_handler.connect_pvs():
            self.status_bar.showMessage("EPICS connection initiated")
        else:
            self.status_bar.showMessage("Failed to connect to EPICS")
            qt.QMessageBox.warning(
                self, "Connection Error",
                "Failed to connect to EPICS PVs.\n"
                "Make sure the EPICS IOC is running:\n"
                "  softIoc -d db/robot.db"
            )

    def _on_connection_changed(self, connected):
        self.control_panel.status_display.set_connected(connected)
        self.control_panel.sample_ops.set_enabled(connected)
        self.gripper_widget.set_enabled(connected)

        if connected:
            self.status_bar.showMessage("Connected to EPICS")
            holder = self.epics_handler.get_value('holder')
            if holder:
                self.control_panel.status_display.set_holder(holder)
        else:
            self.status_bar.showMessage("Disconnected from EPICS")

    def _on_step_changed(self, step):
        self.control_panel.status_display.set_current_step(step)

        # Step 12 = waiting for measurement
        if step == 12:
            self._waiting_for_measurement = True
            self.control_panel.sample_ops.set_waiting_for_measurement(True)
            self.status_bar.showMessage("Waiting for measurement - Press Continue or Abort")
        elif step == 0:
            self._waiting_for_measurement = False
            self.control_panel.sample_ops.set_waiting_for_measurement(False)
            self.control_panel.sample_ops.set_running(False)
            self.status_bar.showMessage("Ready")

        # Show running state
        if step > 0:
            self.control_panel.sample_ops.set_running(True)

    def _on_wait_changed(self, wait_value):
        """Handle wait PV changes."""
        if wait_value == 1:  # Continue
            self._waiting_for_measurement = False
            self.control_panel.sample_ops.set_waiting_for_measurement(False)
            self.status_bar.showMessage("Continuing sequence...")
        elif wait_value == 2:  # Abort
            self._waiting_for_measurement = False
            self.control_panel.sample_ops.set_waiting_for_measurement(False)
            self.status_bar.showMessage("Sequence aborted")

    def _on_gripper_changed(self, is_open):
        self.control_panel.status_display.set_gripper_state(is_open)
        self.gripper_widget.set_gripper_state(is_open)

    def _on_holder_changed(self, holder):
        self.control_panel.status_display.set_holder(holder)

    def _on_mode_changed(self, mode):
        self.control_panel.status_display.set_mode(mode)

    def _put_to_sample(self, holder_num):
        self.epics_handler.set_holder(holder_num)
        self.epics_handler.set_calib_mode(0)
        self.epics_handler.set_start_step(0)
        self.epics_handler.set_wait(0)  # Reset wait
        self.epics_handler.trigger_sequence()
        self.status_bar.showMessage(f"Putting holder {holder_num} to sample holder...")

    def _remove_from_sample(self, holder_num):
        self.epics_handler.set_holder(holder_num)
        self.epics_handler.set_calib_mode(0)
        self.epics_handler.set_start_step(13)
        self.epics_handler.set_wait(1)  # Continue past wait point
        self.epics_handler.trigger_sequence()
        self.status_bar.showMessage(f"Removing sample to holder {holder_num}...")

    def _continue_sequence(self):
        """Continue after measurement wait."""
        self.epics_handler.set_wait(1)
        self.status_bar.showMessage("Continuing sequence...")

    def _abort_sequence(self):
        """Abort and return sample."""
        self.epics_handler.set_wait(2)
        self.status_bar.showMessage("Aborting - returning sample...")

    def _open_calibration(self):
        if self._calibration_window is None:
            self._calibration_window = CalibrationWindow(self.epics_handler, self)
        self._calibration_window.show()
        self._calibration_window.raise_()
        self._calibration_window.activateWindow()

    def _on_emergency_stop(self):
        self.epics_handler.stop_sequence()
        self.status_bar.showMessage("EMERGENCY STOP - Sequence paused")
        qt.QMessageBox.warning(
            self, "Emergency Stop",
            "Sequence has been paused.\n"
            "Click 'Resume' or restart the sequence."
        )

    def closeEvent(self, event):
        self.epics_handler.disconnect()
        if self._calibration_window:
            self._calibration_window.close()
        event.accept()


def main():
    """Main entry point."""
    app = qt.QApplication(sys.argv)
    app.setStyle("Fusion")

    window = RobotControlMainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
