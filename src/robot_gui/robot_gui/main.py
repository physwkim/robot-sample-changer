"""Main application using silx.gui."""

import sys
from silx.gui import qt

from .epics_handler import EpicsHandler
from .control_panel import ControlPanel, STEP_NAMES
from .gripper_widget import GripperWidget
from .calibration_window import CalibrationWindow


class RobotControlMainWindow(qt.QMainWindow):
    """Main window for robot control GUI."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR3e Sample Changer")
        self.setMinimumSize(660, 900)
        self.resize(660, 960)

        self.epics_handler = EpicsHandler(self)
        self._calibration_window = None

        self._setup_ui()
        self._setup_menu()
        self._connect_signals()
        self._connect_epics()

    def _setup_ui(self):
        central_widget = qt.QWidget()
        self.setCentralWidget(central_widget)

        layout = qt.QVBoxLayout(central_widget)
        layout.setSpacing(14)
        layout.setContentsMargins(14, 14, 14, 14)

        self.control_panel = ControlPanel()
        layout.addWidget(self.control_panel)

        self.gripper_widget = GripperWidget()
        layout.addWidget(self.gripper_widget)

        self.status_bar = qt.QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Connecting to EPICS...")

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
        cp = self.control_panel
        cp.mount_requested.connect(self._mount)
        cp.return_requested.connect(self._return)
        cp.map_requested.connect(self._map_holder)
        cp.continue_requested.connect(self._continue_sequence)
        cp.abort_requested.connect(self._abort_sequence)
        cp.pause_toggled.connect(self._pause_toggled)
        cp.recover_requested.connect(self._recover)
        cp.trigger_requested.connect(self._advanced_trigger)
        cp.pause_step_changed.connect(self.epics_handler.set_pause_step)
        cp.open_calibration_requested.connect(self._open_calibration)
        cp.holder_selector.holder_changed.connect(self.epics_handler.set_holder)

        # Gripper
        self.gripper_widget.open_requested.connect(self.epics_handler.open_gripper)
        self.gripper_widget.close_requested.connect(self.epics_handler.close_gripper)

        # EPICS
        eh = self.epics_handler
        eh.connection_changed.connect(self._on_connection_changed)
        eh.current_step_changed.connect(self._on_step_changed)
        eh.gripper_rbv_changed.connect(self._on_gripper_changed)
        eh.holder_changed.connect(self._on_holder_changed)
        eh.calib_mode_changed.connect(self._on_mode_changed)
        eh.stop_changed.connect(self._on_stop_changed)
        eh.pause_step_changed.connect(self.control_panel.advanced.set_pause_step)
        eh.loaded_changed.connect(self._on_loaded_changed)

    def _connect_epics(self):
        self.status_bar.showMessage("Connecting to EPICS...")
        if self.epics_handler.connect_pvs():
            self.status_bar.showMessage("EPICS connection initiated")
        else:
            self.status_bar.showMessage("Failed to connect to EPICS")
            qt.QMessageBox.warning(
                self, "Connection Error",
                "Failed to connect to EPICS PVs.\n"
                "Make sure robot_ioc is running (systemd) and\n"
                "EPICS_CA_NAME_SERVERS points at it."
            )

    def _on_connection_changed(self, connected):
        cp = self.control_panel
        cp.status_display.set_connected(connected)
        cp.sample_ops.set_enabled(connected)
        cp.holder_map.set_enabled(connected)
        cp.advanced.set_enabled(connected)
        self.gripper_widget.set_enabled(connected)

        if connected:
            self.status_bar.showMessage("Connected to EPICS")
            # Read the daemon's world back so the panel starts truthful.
            holder = self.epics_handler.get_value('holder')
            if holder:
                cp.status_display.set_holder(int(holder))
                cp.holder_selector.set_selected_holder(int(holder))
            for name, slot in [
                ('current_step', self._on_step_changed),
                ('calib_mode', self._on_mode_changed),
                ('stop', self._on_stop_changed),
                ('loaded', self._on_loaded_changed),
                ('pause_step', cp.advanced.set_pause_step),
            ]:
                value = self.epics_handler.get_value(name)
                if value is not None:
                    slot(int(value))
        else:
            self.status_bar.showMessage("Disconnected from EPICS")

    def _on_step_changed(self, step):
        self.control_panel.status_display.set_current_step(step)

        # Step 12 is the stage-standby wait: measurement in progress.
        # Continue retrieves the sample, Abort leaves it and stops.
        waiting = step == 12
        self.control_panel.sample_ops.set_waiting(waiting)
        if waiting:
            self.status_bar.showMessage(
                "Sample on stage — Continue to retrieve it, Abort to leave it there"
            )
        elif step == 0:
            self.status_bar.showMessage("Ready")
        else:
            name = STEP_NAMES.get(step, "")
            self.status_bar.showMessage(f"Running: step {step} {name}")

    def _on_gripper_changed(self, is_open):
        self.control_panel.status_display.set_gripper_state(is_open)
        self.gripper_widget.set_gripper_state(is_open)

    def _on_holder_changed(self, holder):
        self.control_panel.status_display.set_holder(holder)

    def _on_mode_changed(self, mode):
        self.control_panel.status_display.set_mode(mode)

    def _on_stop_changed(self, stop):
        paused = bool(stop)
        self.control_panel.status_display.set_motion(paused)
        self.control_panel.sample_ops.set_paused(paused)

    def _on_loaded_changed(self, loaded):
        self.control_panel.status_display.set_loaded(loaded)

    def _confirm_if_busy(self, action):
        """A trigger while CurrentStep > 0 either interleaves with a live
        run or restarts an interrupted one — never silent. True = go."""
        step = self.epics_handler.get_value('current_step')
        if not step or int(step) == 0:
            return True
        name = STEP_NAMES.get(int(step), "")
        answer = qt.QMessageBox.question(
            self, "Sequence not idle",
            f"CurrentStep is {int(step)} ({name}): a sequence is running or "
            f"was interrupted there.\n\n{action} anyway?",
            qt.QMessageBox.Yes | qt.QMessageBox.No,
            qt.QMessageBox.No,
        )
        return answer == qt.QMessageBox.Yes

    def _mount(self, holder_num):
        if not self._confirm_if_busy(f"Mount holder {holder_num}"):
            return
        eh = self.epics_handler
        eh.set_holder(holder_num)
        eh.set_calib_mode(0)
        eh.set_start_step(0)
        eh.set_pause_step(0)
        eh.set_wait(0)
        eh.trigger_sequence()
        self.status_bar.showMessage(f"Mounting holder {holder_num} on the stage...")

    def _return(self, holder_num):
        if not self._confirm_if_busy(f"Return the sample to holder {holder_num}"):
            return
        eh = self.epics_handler
        eh.set_holder(holder_num)
        eh.set_calib_mode(0)
        # From wherever the arm is, step 7 plans a collision-checked move
        # to stage standby; the daemon then waits at step 12 — press
        # Continue there to retrieve. (The daemon resets Wait to 0 at
        # every run start, so pre-setting 1 here would be lost.)
        eh.set_start_step(7)
        eh.set_pause_step(0)
        eh.set_wait(0)
        eh.trigger_sequence()
        self.status_bar.showMessage(
            f"Going to the stage — press Continue at the wait to retrieve "
            f"the sample to holder {holder_num}"
        )

    def _map_holder(self, holder_num, source):
        if not self._confirm_if_busy(f"Map holder {holder_num}"):
            return
        eh = self.epics_handler
        eh.set_holder(holder_num)
        eh.set_map_source(source)
        eh.set_calib_mode(6)
        eh.set_start_step(0)  # holder map refuses mid-sequence resumes
        eh.set_pause_step(0)
        eh.set_wait(0)
        eh.trigger_sequence()
        src = "its own puck" if source == 0 else f"the puck from holder {source}"
        self.status_bar.showMessage(
            f"Mapping holder {holder_num} with {src} — the puck stays seated"
        )

    def _recover(self):
        answer = qt.QMessageBox.question(
            self, "Recover",
            "Unlock a protective stop if any, resend the robot program, and\n"
            "walk the arm back to holder standby. The gripper is not touched.\n\n"
            "Recover now?",
            qt.QMessageBox.Yes | qt.QMessageBox.No,
            qt.QMessageBox.No,
        )
        if answer != qt.QMessageBox.Yes:
            return
        eh = self.epics_handler
        eh.set_calib_mode(4)
        eh.set_wait(0)
        eh.trigger_sequence()
        self.status_bar.showMessage("Recovering to holder standby...")

    def _advanced_trigger(self, mode, start_step):
        if not self._confirm_if_busy("Trigger"):
            return
        eh = self.epics_handler
        eh.set_calib_mode(mode)
        eh.set_start_step(start_step)
        eh.set_wait(0)
        eh.trigger_sequence()
        self.status_bar.showMessage(
            f"Triggered mode {mode} from step {start_step}"
        )

    def _continue_sequence(self):
        self.epics_handler.set_wait(1)
        self.status_bar.showMessage("Continuing — retrieving the sample...")

    def _abort_sequence(self):
        self.epics_handler.set_wait(2)
        self.epics_handler.clear_trigger()
        self.status_bar.showMessage("Stopped at the wait — sample left on the stage")

    def _pause_toggled(self, paused):
        if paused:
            self.epics_handler.stop_sequence()
            self.status_bar.showMessage("Pause requested — stops after the current step")
        else:
            self.epics_handler.resume_sequence()
            self.status_bar.showMessage("Resumed")

    def _open_calibration(self):
        if self._calibration_window is None:
            self._calibration_window = CalibrationWindow(self.epics_handler, self)
        self._calibration_window.show()
        self._calibration_window.raise_()
        self._calibration_window.activateWindow()

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
