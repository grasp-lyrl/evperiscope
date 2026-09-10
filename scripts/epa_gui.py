#!/usr/bin/env python3
"""EPA flight controller GUI — a PyQt5 command panel.

Flight commands go to the C++ epa_interface node over /epa/cmd/*; MAVROS is
read directly for status display only. The MAVROS topics live under the
`robot_namespace` parameter (default "kestrel1"), which must match the one
given to epa_interface.
"""

import sys
import threading

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QGridLayout,
)
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, EstimatorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Twist
from std_srvs.srv import Trigger


class GuiNode(Node):
    """Minimal ROS2 node for the GUI — subscribes to state, publishes commands."""

    def __init__(self):
        super().__init__('epa_gui')

        ns = '/' + self.declare_parameter('robot_namespace', 'kestrel1').value

        # --- Status subscriptions (display only) ---
        self.state_sub = self.create_subscription(
            State, f'{ns}/mavros/state', self._state_cb, 10)
        self.estimator_sub = self.create_subscription(
            EstimatorStatus, f'{ns}/mavros/estimator_status', self._estimator_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, f'{ns}/mavros/local_position/odom', self._odom_cb, 10)

        # --- Command publishers (to epa_interface) ---
        self.takeoff_pub = self.create_publisher(
            Float64, '/epa/cmd/takeoff', 10)
        self.nudge_pub = self.create_publisher(
            Vector3, '/epa/cmd/position_nudge', 10)
        self.vel_pub = self.create_publisher(
            Twist, '/epa/cmd/velocity', 10)

        # --- Service Clients ---
        self.cli_arm = self.create_client(Trigger, '/epa/cmd/arm')
        self.cli_disarm = self.create_client(Trigger, '/epa/cmd/disarm')
        self.cli_offboard = self.create_client(Trigger, '/epa/cmd/offboard')
        self.cli_land = self.create_client(Trigger, '/epa/cmd/land')
        self.cli_tracking = self.create_client(Trigger, '/epa/cmd/tracking')
        self.cli_position = self.create_client(Trigger, '/epa/cmd/position')

        # Display state
        self.current_state = State()
        self.vio_healthy = False
        self.current_odom = None

    # --- Status callbacks ---

    def _state_cb(self, msg):
        self.current_state = msg

    def _estimator_cb(self, msg):
        self.vio_healthy = (
            (msg.pos_horiz_rel_status_flag or msg.pos_horiz_abs_status_flag)
            and not msg.accel_error_status_flag
        )

    def _odom_cb(self, msg):
        self.current_odom = msg

    def _call_trigger_service(self, client, name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service {name} not available')
            return

        req = Trigger.Request()
        future = client.call_async(req)

        def done_callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(f'{name} -> Success: {response.message}')
                else:
                    self.get_logger().warn(f'{name} -> Failed: {response.message}')
            except Exception as e:
                self.get_logger().error(f'Service call {name} failed: {e}')

        future.add_done_callback(done_callback)

    # --- Command methods ---

    def arm(self):
        self._call_trigger_service(self.cli_arm, 'arm')

    def disarm(self):
        self._call_trigger_service(self.cli_disarm, 'disarm')

    def offboard(self):
        self._call_trigger_service(self.cli_offboard, 'offboard')

    def land(self):
        self._call_trigger_service(self.cli_land, 'land')

    def tracking(self):
        self._call_trigger_service(self.cli_tracking, 'tracking')

    def position(self):
        self._call_trigger_service(self.cli_position, 'position')

    def takeoff(self, height):
        msg = Float64()
        msg.data = height
        self.takeoff_pub.publish(msg)
        self.get_logger().info(f'Sent takeoff: height={height:.2f}')

    def update_setpoint_relative(self, dx, dy, dz):
        msg = Vector3()
        msg.x = float(dx)
        msg.y = float(dy)
        msg.z = float(dz)
        self.nudge_pub.publish(msg)

    def set_velocity(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.vel_pub.publish(msg)


class ControllerGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(100)

    def initUI(self):
        self.setWindowTitle('EPA Flight Controller')

        main_layout = QVBoxLayout()

        # Status
        self.status_label = QLabel('Status: Disconnected')
        main_layout.addWidget(self.status_label)

        self.mode_label = QLabel('Mode: Unknown')
        main_layout.addWidget(self.mode_label)

        self.vio_label = QLabel('VIO: Unknown')
        main_layout.addWidget(self.vio_label)

        # Arming
        arm_layout = QHBoxLayout()
        self.btn_arm = QPushButton('ARM')
        self.btn_arm.setStyleSheet('background-color: green; color: white')
        self.btn_arm.clicked.connect(self.node.arm)
        arm_layout.addWidget(self.btn_arm)

        self.btn_disarm = QPushButton('DISARM')
        self.btn_disarm.setStyleSheet('background-color: red; color: white')
        self.btn_disarm.clicked.connect(self.node.disarm)
        arm_layout.addWidget(self.btn_disarm)

        main_layout.addLayout(arm_layout)

        # Mode buttons
        mode_layout = QHBoxLayout()
        self.btn_offboard = QPushButton('OFFBOARD')
        self.btn_offboard.clicked.connect(self.node.offboard)
        mode_layout.addWidget(self.btn_offboard)

        self.btn_position = QPushButton('POSITION')
        self.btn_position.setStyleSheet('background-color: purple; color: white')
        self.btn_position.clicked.connect(self.node.position)
        mode_layout.addWidget(self.btn_position)

        self.btn_tracking = QPushButton('TRACKING')
        self.btn_tracking.setStyleSheet('background-color: blue; color: white')
        self.btn_tracking.clicked.connect(self.node.tracking)
        mode_layout.addWidget(self.btn_tracking)

        main_layout.addLayout(mode_layout)

        # Land
        self.btn_land = QPushButton('LAND')
        self.btn_land.setStyleSheet('background-color: orange; color: black')
        self.btn_land.clicked.connect(self.node.land)
        main_layout.addWidget(self.btn_land)

        # Takeoff
        takeoff_layout = QHBoxLayout()
        takeoff_layout.addWidget(QLabel('Height:'))
        self.txt_height = QLineEdit('1.5')
        takeoff_layout.addWidget(self.txt_height)

        self.btn_takeoff = QPushButton('TAKEOFF')
        self.btn_takeoff.clicked.connect(self.on_takeoff)
        takeoff_layout.addWidget(self.btn_takeoff)
        main_layout.addLayout(takeoff_layout)

        # Relative control grid
        control_layout = QGridLayout()
        control_layout.addWidget(QLabel('Position (Body Frame)'), 0, 1, 1, 3)

        self.btn_xp = QPushButton('X +0.1')
        self.btn_xp.clicked.connect(lambda: self.node.update_setpoint_relative(0.1, 0, 0))
        control_layout.addWidget(self.btn_xp, 1, 1)

        self.btn_xm = QPushButton('X -0.1')
        self.btn_xm.clicked.connect(lambda: self.node.update_setpoint_relative(-0.1, 0, 0))
        control_layout.addWidget(self.btn_xm, 2, 1)

        self.btn_yp = QPushButton('Y +0.1')
        self.btn_yp.clicked.connect(lambda: self.node.update_setpoint_relative(0, 0.1, 0))
        control_layout.addWidget(self.btn_yp, 1, 2)

        self.btn_ym = QPushButton('Y -0.1')
        self.btn_ym.clicked.connect(lambda: self.node.update_setpoint_relative(0, -0.1, 0))
        control_layout.addWidget(self.btn_ym, 2, 2)

        self.btn_zp = QPushButton('Z +0.1')
        self.btn_zp.clicked.connect(lambda: self.node.update_setpoint_relative(0, 0, 0.1))
        control_layout.addWidget(self.btn_zp, 1, 3)

        self.btn_zm = QPushButton('Z -0.1')
        self.btn_zm.clicked.connect(lambda: self.node.update_setpoint_relative(0, 0, -0.1))
        control_layout.addWidget(self.btn_zm, 2, 3)

        main_layout.addLayout(control_layout)

        # Velocity grid
        vel_layout = QGridLayout()
        vel_layout.addWidget(QLabel('Velocity (Body Frame)'), 0, 1, 1, 3)

        self.btn_vx_p = QPushButton('Vx +0.1')
        self.btn_vx_p.pressed.connect(lambda: self.node.set_velocity(0.1, 0.0, 0.0))
        self.btn_vx_p.released.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.0))
        vel_layout.addWidget(self.btn_vx_p, 1, 1)

        self.btn_vx_m = QPushButton('Vx -0.1')
        self.btn_vx_m.pressed.connect(lambda: self.node.set_velocity(-0.1, 0.0, 0.0))
        self.btn_vx_m.released.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.0))
        vel_layout.addWidget(self.btn_vx_m, 2, 1)

        self.btn_vy_p = QPushButton('Vy +0.1')
        self.btn_vy_p.pressed.connect(lambda: self.node.set_velocity(0.0, 0.1, 0.0))
        self.btn_vy_p.released.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.0))
        vel_layout.addWidget(self.btn_vy_p, 1, 2)

        self.btn_vy_m = QPushButton('Vy -0.1')
        self.btn_vy_m.pressed.connect(lambda: self.node.set_velocity(0.0, -0.1, 0.0))
        self.btn_vy_m.released.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.0))
        vel_layout.addWidget(self.btn_vy_m, 2, 2)

        self.btn_vz_p = QPushButton('Vz +0.1')
        self.btn_vz_p.pressed.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.1))
        self.btn_vz_p.released.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.0))
        vel_layout.addWidget(self.btn_vz_p, 1, 3)

        self.btn_vz_m = QPushButton('Vz -0.1')
        self.btn_vz_m.pressed.connect(lambda: self.node.set_velocity(0.0, 0.0, -0.1))
        self.btn_vz_m.released.connect(lambda: self.node.set_velocity(0.0, 0.0, 0.0))
        vel_layout.addWidget(self.btn_vz_m, 2, 3)

        main_layout.addLayout(vel_layout)

        self.setLayout(main_layout)

    def update_status(self):
        state = self.node.current_state
        self.status_label.setText(f'Connected: {state.connected}, Armed: {state.armed}')
        self.mode_label.setText(f'Mode: {state.mode}')

        if self.node.vio_healthy:
            self.vio_label.setText('VIO: OK')
            self.vio_label.setStyleSheet('color: green')
        else:
            self.vio_label.setText('VIO: Unhealthy/Initializing')
            self.vio_label.setStyleSheet('color: orange')

    def on_takeoff(self):
        try:
            h = float(self.txt_height.text())
            self.node.takeoff(h)
        except ValueError:
            pass


def main(args=None):
    rclpy.init(args=args)

    gui_node = GuiNode()

    app = QApplication(sys.argv)
    gui = ControllerGUI(gui_node)
    gui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
