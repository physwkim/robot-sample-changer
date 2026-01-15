"""EPICS PV Handler using pyepics."""

import epics
from silx.gui import qt


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
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pvs = {}
        self.connected = False
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
            if value is not None and name in signal_map:
                # Use Qt's thread-safe signal emission
                signal_map[name].emit(int(value))

        return callback

    def _connection_callback(self, pvname=None, conn=None, **kwargs):
        """Handle connection state changes."""
        self._check_connection()

    def _check_connection(self):
        """Check if all PVs are connected."""
        if not self.pvs:
            return

        all_connected = all(pv.connected for pv in self.pvs.values())

        if all_connected != self.connected:
            self.connected = all_connected
            self.connection_changed.emit(all_connected)

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

    def set_calib_mode(self, mode):
        return self.set_value('calib_mode', mode)

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

    def disconnect(self):
        """Disconnect all PVs."""
        self._connection_timer.stop()
        for pv in self.pvs.values():
            pv.disconnect()
        self.pvs.clear()
        self.connected = False
