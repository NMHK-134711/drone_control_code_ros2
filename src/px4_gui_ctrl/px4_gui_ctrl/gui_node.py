#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QLineEdit, QGroupBox, QGridLayout)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_gui_node')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 상태 수신 (Subscribe)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_profile)

        # 명령 송신 (Publish)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 드론 상태 변수
        self.px4_timestamp = 0
        self.is_armed = False
        self.nav_state = 0
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.current_yaw = 0.0 # 현재 바라보는 방향(라디안)
        
        # 목표 좌표 변수 (초기값: 5m 상공, 앞(0도)을 바라봄)
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, -5.0
        self.target_yaw = 0.0 

        # Offboard 유지를 위한 10Hz 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)

    def status_callback(self, msg):
        self.px4_timestamp = msg.timestamp
        self.is_armed = (msg.arming_state == 2) # 2 = ARMED
        self.nav_state = msg.nav_state

    def pos_callback(self, msg):
        # NED 좌표계 기준 현재 위치 및 방향
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading # heading 값이 현재 Yaw(라디안)입니다.

    def timer_callback(self):
        if self.px4_timestamp == 0:
            return
        # 오프보드 모드 유지를 위해 목표 좌표를 끊임없이 쏨
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.px4_timestamp
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw # 하드코딩되었던 0.0 대신 목표 Yaw 변수 대입!
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yawspeed = float('nan')
        msg.timestamp = self.px4_timestamp
        self.trajectory_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        msg.timestamp = self.px4_timestamp
        self.command_pub.publish(msg)


class DroneGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

        # UI 업데이트용 타이머 (ROS 콜백과 별개로 20Hz로 화면 새로고침)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(50)

    def initUI(self):
        self.setWindowTitle('드론 정밀 제어 GCS (방향 제어 포함)')
        self.resize(500, 300) # Yaw 칸이 생겼으니 창을 조금 넓힙니다.
        main_layout = QVBoxLayout()

        # 1. 텔레메트리 (상태 표시) 패널
        status_group = QGroupBox("실시간 상태 (Telemetry)")
        status_layout = QGridLayout()
        
        self.lbl_armed = QLabel("시동 상태: 대기중")
        self.lbl_armed.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_pos = QLabel("현재 위치: X: 0.00 | Y: 0.00 | Z: 0.00 | Yaw: 0°")
        
        status_layout.addWidget(self.lbl_armed, 0, 0)
        status_layout.addWidget(self.lbl_pos, 1, 0)
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # 2. 버튼 컨트롤 패널
        ctrl_group = QGroupBox("명령 컨트롤")
        ctrl_layout = QHBoxLayout()
        
        btn_arm = QPushButton("시동 (Arm)")
        btn_arm.setStyleSheet("background-color: #ffcccc;")
        btn_arm.clicked.connect(lambda: self.node.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0))
        
        btn_disarm = QPushButton("시동 끄기 (Disarm)")
        btn_disarm.clicked.connect(lambda: self.node.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0))
        
        btn_offboard = QPushButton("오프보드 비행 시작")
        btn_offboard.setStyleSheet("background-color: #ccffcc;")
        btn_offboard.clicked.connect(lambda: self.node.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0))

        ctrl_layout.addWidget(btn_arm)
        ctrl_layout.addWidget(btn_disarm)
        ctrl_layout.addWidget(btn_offboard)
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)

        # 3. 좌표 및 방향 이동 패널
        pos_group = QGroupBox("정밀 목표 설정 (NED 및 각도)")
        pos_layout = QHBoxLayout()
        
        self.input_x = QLineEdit("0.0")
        self.input_y = QLineEdit("0.0")
        self.input_z = QLineEdit("-5.0")
        self.input_yaw = QLineEdit("0.0") # Yaw 입력창 추가
        
        btn_move = QPushButton("전송")
        btn_move.clicked.connect(self.update_target_position)

        pos_layout.addWidget(QLabel("X(m):"))
        pos_layout.addWidget(self.input_x)
        pos_layout.addWidget(QLabel("Y(m):"))
        pos_layout.addWidget(self.input_y)
        pos_layout.addWidget(QLabel("Z(m):"))
        pos_layout.addWidget(self.input_z)
        pos_layout.addWidget(QLabel("Yaw(도):"))
        pos_layout.addWidget(self.input_yaw)
        pos_layout.addWidget(btn_move)
        
        pos_group.setLayout(pos_layout)
        main_layout.addWidget(pos_group)

        self.setLayout(main_layout)

    def update_target_position(self):
        try:
            self.node.target_x = float(self.input_x.text())
            self.node.target_y = float(self.input_y.text())
            self.node.target_z = float(self.input_z.text())
            
            # 사람이 입력한 각도(Degree)를 드론이 알아듣는 라디안(Radian)으로 변환
            yaw_deg = float(self.input_yaw.text())
            self.node.target_yaw = math.radians(yaw_deg)
            
            self.node.get_logger().info(f"좌표 업데이트: X={self.node.target_x}, Y={self.node.target_y}, Z={self.node.target_z}, Yaw={yaw_deg}°")
        except ValueError:
            self.node.get_logger().error("숫자만 입력해주세요!")

    def update_ui(self):
        # ROS 2 노드가 계속 돌아가게 스핀
        rclpy.spin_once(self.node, timeout_sec=0)
        
        # UI 텍스트 갱신
        arm_text = "ARMED (위험!)" if self.node.is_armed else "DISARMED (안전)"
        color = "red" if self.node.is_armed else "blue"
        self.lbl_armed.setText(f"시동 상태: <span style='color:{color}'>{arm_text}</span>")
        
        # 드론이 보내주는 라디안 값을 다시 사람이 보기 편하게 각도로 변환하여 출력
        current_yaw_deg = math.degrees(self.node.current_yaw)
        self.lbl_pos.setText(f"현재 위치: X: {self.node.current_x:.2f} | Y: {self.node.current_y:.2f} | Z: {self.node.current_z:.2f} | Yaw: {current_yaw_deg:.0f}°")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    node = DroneControlNode()
    gui = DroneGUI(node)
    gui.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
