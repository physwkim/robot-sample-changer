"""Gripper Control Widget using silx.gui - Buttons only."""

from silx.gui import qt


class GripperWidget(qt.QGroupBox):
    """Gripper control widget with Open/Close buttons."""

    open_requested = qt.Signal()
    close_requested = qt.Signal()

    def __init__(self, parent=None):
        super().__init__("Gripper Control", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = qt.QHBoxLayout(self)
        layout.setSpacing(12)

        self.open_btn = qt.QPushButton("Open")
        self.open_btn.setMinimumSize(100, 50)
        self.open_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 6px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:pressed { background-color: #3d8b40; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.open_btn.clicked.connect(self.open_requested.emit)
        layout.addWidget(self.open_btn)

        self.close_btn = qt.QPushButton("Close")
        self.close_btn.setMinimumSize(100, 50)
        self.close_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border-radius: 6px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:pressed { background-color: #E65100; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.close_btn.clicked.connect(self.close_requested.emit)
        layout.addWidget(self.close_btn)

    def set_gripper_state(self, is_open):
        """Update button styles based on gripper state."""
        if is_open:
            self.open_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2E7D32;
                    color: white;
                    border: 3px solid #4CAF50;
                    border-radius: 6px;
                    font-size: 14px;
                    font-weight: bold;
                }
            """)
            self.close_btn.setStyleSheet("""
                QPushButton {
                    background-color: #FF9800;
                    color: white;
                    border-radius: 6px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #F57C00; }
                QPushButton:disabled { background-color: #cccccc; }
            """)
        else:
            self.open_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border-radius: 6px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #45a049; }
                QPushButton:disabled { background-color: #cccccc; }
            """)
            self.close_btn.setStyleSheet("""
                QPushButton {
                    background-color: #E65100;
                    color: white;
                    border: 3px solid #FF9800;
                    border-radius: 6px;
                    font-size: 14px;
                    font-weight: bold;
                }
            """)

    def set_enabled(self, enabled):
        self.open_btn.setEnabled(enabled)
        self.close_btn.setEnabled(enabled)
