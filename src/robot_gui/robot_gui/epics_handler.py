"""EPICS PV Handler using pyepics."""

import time

import epics
from silx.gui import qt

# Robot:State codes. The daemon's DaemonState owns the numbering.
STATE_NAMES = {
    0: "idle",
    1: "running",
    2: "measurement wait",
    3: "paused",
    4: "hold",
}


class EpicsHandler(qt.QObject):
    """Handle EPICS PV connections using pyepics."""

    # Signals for PV value changes
    trigger_changed = qt.Signal(int)
    holder_changed = qt.Signal(int)
    wait_changed = qt.Signal(int)
    stop_changed = qt.Signal(int)
    current_step_changed = qt.Signal(int)
    gripper_changed = qt.Signal(int)
    gripper_rbv_changed = qt.Signal(int)
    calib_mode_changed = qt.Signal(int)
    pause_step_changed = qt.Signal(int)
    loaded_changed = qt.Signal(int)
    connection_changed = qt.Signal(bool)

    # PV names
    PV_NAMES = {
        'trigger': 'Robot:Trigger',
        'holder': 'Robot:Holder',
        'wait': 'Robot:Wait',
        'stop': 'Robot:Stop',
        'current_step': 'Robot:CurrentStep',
        'gripper': 'Robot:Gripper',
        'gripper_rbv': 'Robot:Gripper_RBV',
        'calib_mode': 'Robot:CalibMode',
        'pause_step': 'Robot:PauseStep',
        'start_step': 'Robot:StartStep',
        'loaded': 'Robot:Loaded',
        'map_source': 'Robot:MapSource',
        'jog_x': 'Robot:JogX',
        'jog_y': 'Robot:JogY',
        'jog_z': 'Robot:JogZ',
        'jog_step': 'Robot:JogStep',
        'state': 'Robot:State',
        'alive': 'Robot:Alive',
    }

    # Records the sequencer publishes only where the IOC serves them.
    # An IOC still running an older db/robot.db has neither, and the
    # daemon runs perfectly well against it -- so their absence must not
    # read as "this GUI is disconnected", and must not take the run
    # buttons with it.
    OPTIONAL_PVS = ('state', 'alive')

    # How long Robot:Alive may stand still before a daemon that promised
    # beats is treated as gone. It beats every service pass, 10 Hz in
    # every standing loop, so this is twenty missed beats.
    BEAT_STALE_S = 2.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pvs = {}
        self.connected = False
        # The last Robot:Alive value seen and when this GUI saw it, by
        # its own clock: the IOC's timestamp would need the two hosts to
        # agree, and what is being measured is "did an update arrive".
        self._beat = None
        self._connection_timer = qt.QTimer(self)
        self._connection_timer.timeout.connect(self._check_connection)

    def connect_pvs(self):
        """Connect to all EPICS PVs using pyepics."""
        try:
            for name, pv_name in self.PV_NAMES.items():
                pv = epics.PV(
                    pv_name,
                    callback=self._create_callback(name),
                    connection_callback=self._connection_callback
                )
                self.pvs[name] = pv

            self._connection_timer.start(1000)
            return True
        except Exception as e:
            print(f"Failed to connect to EPICS PVs: {e}")
            return False

    def _create_callback(self, name):
        """Create pyepics callback for value changes."""
        signal_map = {
            'trigger': self.trigger_changed,
            'holder': self.holder_changed,
            'wait': self.wait_changed,
            'stop': self.stop_changed,
            'current_step': self.current_step_changed,
            'gripper': self.gripper_changed,
            'gripper_rbv': self.gripper_rbv_changed,
            'calib_mode': self.calib_mode_changed,
            'pause_step': self.pause_step_changed,
            'loaded': self.loaded_changed,
        }

        def callback(pvname=None, value=None, **kwargs):
            if value is None:
                return
            if name == 'alive':
                self._note_beat(int(value))
            if name in signal_map:
                # Use Qt's thread-safe signal emission
                signal_map[name].emit(int(value))

        return callback

    def _note_beat(self, value):
        """Record when Robot:Alive last changed."""
        if self._beat is None or self._beat[0] != value:
            self._beat = (value, time.monotonic())

    def _connection_callback(self, pvname=None, conn=None, **kwargs):
        """Handle connection state changes."""
        self._check_connection()

    def _check_connection(self):
        """Check if all PVs are connected."""
        if not self.pvs:
            return

        all_connected = all(
            pv.connected
            for name, pv in self.pvs.items()
            if name not in self.OPTIONAL_PVS
        )

        if all_connected != self.connected:
            self.connected = all_connected
            self.connection_changed.emit(all_connected)

    def not_ready_for_a_run(self):
        """Why a trigger cannot start a run right now, or None when it
        can.

        A run begins at the idle trigger wait and nowhere else: the
        daemon reads Robot:Trigger there, and drops whatever it finds in
        the record when any other wait opens. So a press made while the
        arm is moving, or while a measurement wait is standing, writes a
        record nobody will read.

        Robot:State names the loop and Robot:Alive counts its service
        passes; neither alone is enough, since a state that is not being
        re-stamped is the stale reading this exists to catch.
        """
        pv = self.pvs.get('state')
        value = pv.get() if pv is not None and pv.connected else None
        if value is None:
            # No record to ask (see OPTIONAL_PVS). Nothing is known
            # about the daemon here, so nothing is claimed: refusing
            # would take the fallback GUI down over a missing label.
            return None
        state = int(value)
        if state == 1:
            return "the daemon is moving and reads no commands until it stops"
        beating = (
            self._beat is not None
            and time.monotonic() - self._beat[1] < self.BEAT_STALE_S
        )
        if not beating:
            return (
                "the daemon is not responding — it last said "
                f"{STATE_NAMES.get(state, state)}"
            )
        if state != 0:
            return (
                "a run starts at the idle wait, and the daemon is in the "
                f"{STATE_NAMES.get(state, state)}"
            )
        return None

    def get_value(self, name):
        """Get current value using pyepics."""
        pv = self.pvs.get(name)
        if pv and pv.connected:
            return pv.get()
        return None

    def set_value(self, name, value):
        """Set value using pyepics."""
        pv = self.pvs.get(name)
        if pv and pv.connected:
            pv.put(value)
            return True
        return False

    # Convenience methods
    def set_holder(self, holder_num):
        return self.set_value('holder', holder_num)

    def trigger_sequence(self):
        return self.set_value('trigger', 1)

    def clear_trigger(self):
        return self.set_value('trigger', 0)

    def set_calib_mode(self, mode):
        return self.set_value('calib_mode', mode)

    def set_map_source(self, holder_num):
        return self.set_value('map_source', holder_num)

    def set_wait(self, value):
        return self.set_value('wait', value)

    def stop_sequence(self):
        return self.set_value('stop', 1)

    def resume_sequence(self):
        return self.set_value('stop', 0)

    def open_gripper(self):
        return self.set_value('gripper', 1)

    def close_gripper(self):
        return self.set_value('gripper', 0)

    def set_pause_step(self, step):
        return self.set_value('pause_step', step)

    def set_start_step(self, step):
        return self.set_value('start_step', step)

    def set_jog(self, axis, direction, step_mm):
        """Set jog command for TCP relative move during calibration.

        Args:
            axis: 'x', 'y', or 'z'
            direction: -1 or +1
            step_mm: step size in mm
        """
        self.set_value('jog_step', float(step_mm))
        if axis == 'x':
            return self.set_value('jog_x', direction)
        elif axis == 'y':
            return self.set_value('jog_y', direction)
        elif axis == 'z':
            return self.set_value('jog_z', direction)
        return False

    def disconnect(self):
        """Disconnect all PVs."""
        self._connection_timer.stop()
        for pv in self.pvs.values():
            pv.disconnect()
        self.pvs.clear()
        self.connected = False
