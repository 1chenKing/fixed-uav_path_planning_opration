#!/usr/bin/env python3
import signal
import sys

import rospy
from python_qt_binding.QtWidgets import QApplication

from mission_ui.swarm_control_plugin import SwarmControlWidget


def main():
    rospy.init_node("mission_ground_station", anonymous=False)
    app = QApplication(sys.argv)
    app.setApplicationName("固定翼集群地面站")
    widget = SwarmControlWidget()
    widget.setWindowTitle("固定翼集群地面站")
    widget.resize(1400, 900)
    widget.show()

    def _shutdown(*_args):
        try:
            widget.shutdown_plugin()
        except Exception:
            pass
        try:
            rospy.signal_shutdown("Ground station closed")
        except Exception:
            pass
        app.quit()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    app.aboutToQuit.connect(_shutdown)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
