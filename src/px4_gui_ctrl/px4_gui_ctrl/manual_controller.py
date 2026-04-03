import sys
import math
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class ManualControllerNode(Node):
    def __init__(self):
        super().__init__('manual_controller_node')
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 목표 좌표 초기화 (X, Y, Z, Yaw)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0  # 이륙 전 바닥
        self.target_yaw = 0.0

        # 10Hz로 오프보드 하트비트와 목표 좌표를 쏘는 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 1. 오프보드 모드 하트비트
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode_msg)

        # 2. 목표 좌표(Trajectory Setpoint) 퍼블리시
        sp_msg = TrajectorySetpoint()
        sp_msg.position = [self.target_x, self.target_y, self.target_z]
        sp_msg.yaw = self.target_yaw
        sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(sp_msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """ PX4에 시스템 명령(Arm, 모드 변경 등)을 보내는 함수 """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def arm_and_takeoff(self):
        """ 이륙 명령: 고도를 -1.5m로 설정하고 시동을 걺 """
        self.target_z = -1.5
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard 모드 변경
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # Arm (시동)

class JogControllerUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.step = 0.5  # 한 번 누를 때 이동할 거리 (0.5m)
        self.init_ui()
        
        # PyQt UI가 멈추지 않도록 ROS 2 콜백을 주기적으로 실행하는 타이머
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10) # 10ms

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def init_ui(self):
        self.setWindowTitle('드론 수동 제어 패드')
        self.setGeometry(100, 100, 350, 400)

        main_layout = QVBoxLayout()
        grid = QGridLayout()

        # 상태 표시 라벨
        self.status_label = QLabel("현재 목표 좌표\nX: 0.0 | Y: 0.0 | Z: 0.0 | Yaw: 0°")
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)

        # 컨트롤 버튼 생성
        btn_takeoff = QPushButton("🚀 이륙 (Arm & Offboard)")
        btn_takeoff.clicked.connect(self.action_takeoff)
        main_layout.addWidget(btn_takeoff)

        # 상/하 조작
        btn_up = QPushButton("⬆️ 상승")
        btn_down = QPushButton("⬇️ 하강")
        btn_up.clicked.connect(lambda: self.move_z(-self.step)) # Z 음수가 위쪽!
        btn_down.clicked.connect(lambda: self.move_z(self.step))

        # 전/후/좌/우 조작
        btn_forward = QPushButton("W (앞)")
        btn_backward = QPushButton("S (뒤)")
        btn_left = QPushButton("A (좌)")
        btn_right = QPushButton("D (우)")
        
        btn_forward.clicked.connect(lambda: self.move_horizontal(self.step, 0))
        btn_backward.clicked.connect(lambda: self.move_horizontal(-self.step, 0))
        btn_left.clicked.connect(lambda: self.move_horizontal(0, -self.step))
        btn_right.clicked.connect(lambda: self.move_horizontal(0, self.step))

        # 회전 조작
        btn_yaw_left = QPushButton("↺ 좌회전(90°)")
        btn_yaw_right = QPushButton("↻ 우회전(90°)")
        btn_yaw_left.clicked.connect(lambda: self.rotate_yaw(-math.pi / 2))
        btn_yaw_right.clicked.connect(lambda: self.rotate_yaw(math.pi / 2))

        # 그리드 배치 (십자키 모양)
        grid.addWidget(btn_up, 0, 1)
        grid.addWidget(btn_down, 4, 1)
        
        grid.addWidget(btn_forward, 1, 1)
        grid.addWidget(btn_left, 2, 0)
        grid.addWidget(btn_right, 2, 2)
        grid.addWidget(btn_backward, 3, 1)

        grid.addWidget(btn_yaw_left, 1, 0)
        grid.addWidget(btn_yaw_right, 1, 2)

        main_layout.addLayout(grid)
        self.setLayout(main_layout)

    def update_label(self):
        deg = math.degrees(self.node.target_yaw) % 360
        self.status_label.setText(f"현재 목표 좌표\nX: {self.node.target_x:.2f} | Y: {self.node.target_y:.2f} | Z: {self.node.target_z:.2f} | Yaw: {deg:.0f}°")

    def action_takeoff(self):
        self.node.arm_and_takeoff()
        self.update_label()

    def move_z(self, dz):
        self.node.target_z += dz
        self.update_label()

    def move_horizontal(self, dx_body, dy_body):
        """ 드론이 바라보는 방향(Yaw)을 기준으로 글로벌 좌표 계산 """
        yaw = self.node.target_yaw
        
        # 회전 변환 행렬 (Rotation Matrix) 적용
        # 글로벌 X 변화량 = 앞뒤 이동 * cos(yaw) - 좌우 이동 * sin(yaw)
        self.node.target_x += (dx_body * math.cos(yaw)) - (dy_body * math.sin(yaw))
        # 글로벌 Y 변화량 = 앞뒤 이동 * sin(yaw) + 좌우 이동 * cos(yaw)
        self.node.target_y += (dx_body * math.sin(yaw)) + (dy_body * math.cos(yaw))
        
        self.update_label()

    def rotate_yaw(self, dyaw):
        self.node.target_yaw += dyaw
        self.update_label()

def main(args=None):
    rclpy.init(args=args)
    node = ManualControllerNode()
    
    app = QApplication(sys.argv)
    gui = JogControllerUI(node)
    gui.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()