import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import math
import tkinter as tk
from tkinter import messagebox

# PX4 메시지 임포트 (v1.16 규격)
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class DroneMasterController(Node):
    def __init__(self):
        super().__init__('drone_master_controller')

        # 1. QoS 설정 (PX4 통신은 Best Effort 권장)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Publisher 설정
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 3. Subscriber 설정 (상태 및 위치 모니터링 - _v1 버전 적용)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        # 4. 상태 변수
        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        self.arming_state = 0 # 1: DISARMED, 2: ARMED
        self.nav_state = 0

        # 5. 목표 변수 (기본 이륙 고도 -2m 설정)
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, -2.0
        self.target_yaw = 0.0
        self.step_size = 0.2 # 키보드 1회당 이동 거리(m)

        # 6. 주기적 명령 송신 타이머 (10Hz)
        self.create_timer(0.1, self.timer_callback)

    def pos_callback(self, msg):
        self.curr_x, self.curr_y, self.curr_z = msg.x, msg.y, msg.z
        self.curr_yaw = msg.heading

    def status_callback(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def timer_callback(self):
        # Offboard 모드 유지를 위해 지속적으로 메시지 발행
        self.publish_offboard_mode()
        self.publish_setpoint()

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)]
        msg.yaw = self.target_yaw * (math.pi / 180.0)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, command, p1=0.0, p2=0.0):
        """ PX4에 VehicleCommand를 보내는 공통 함수 """
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = command, p1, p2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

# --- GUI 레이아웃 클래스 ---
class DroneGui:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("PX4 Master GCS")
        self.root.geometry("400x650")

        # [1] 실시간 상태 표시 (Status)
        status_frame = tk.LabelFrame(self.root, text="Drone Status", padx=15, pady=15, fg="darkblue", font=("Arial", 10, "bold"))
        status_frame.pack(pady=10, fill="x", padx=10)

        self.lbl_pos = tk.Label(status_frame, text="X: 0.00 | Y: 0.00 | Z: 0.00", font=("Consolas", 12))
        self.lbl_pos.pack()
        
        self.lbl_arm = tk.Label(status_frame, text="DISARMED", font=("Arial", 14, "bold"), fg="red")
        self.lbl_arm.pack()

        # [2] 목표 좌표 표시 (Targets)
        target_frame = tk.LabelFrame(self.root, text="Target Setpoints", padx=15, pady=15)
        target_frame.pack(pady=10, fill="x", padx=10)
        
        self.lbl_target = tk.Label(target_frame, text="Target -> X: 0.0 | Y: 0.0 | Z: -2.0", font=("Arial", 10))
        self.lbl_target.pack()
        self.lbl_yaw = tk.Label(target_frame, text="Target Yaw: 0.0°", font=("Arial", 10))
        self.lbl_yaw.pack()

        # [3] 제어 버튼 (Command Buttons)
        cmd_frame = tk.LabelFrame(self.root, text="Commands", padx=15, pady=15)
        cmd_frame.pack(pady=10, fill="x", padx=10)

        # ARM & DISARM 버튼
        tk.Button(cmd_frame, text="1. ARM (시동)", command=self.cmd_arm, bg="#FF8C00", fg="white", height=2, width=15).grid(row=0, column=0, padx=5, pady=5)
        tk.Button(cmd_frame, text="2. OFFBOARD", command=self.cmd_offboard, bg="#4682B4", fg="white", height=2, width=15).grid(row=0, column=1, padx=5, pady=5)
        
        # DISARM & LAND 버튼
        tk.Button(cmd_frame, text="DISARM (정지)", command=self.cmd_disarm, bg="#A9A9A9", height=2, width=15).grid(row=1, column=0, padx=5, pady=5)
        tk.Button(cmd_frame, text="LAND (착륙)", command=self.cmd_land, bg="#8FBC8F", height=2, width=15).grid(row=1, column=1, padx=5, pady=5)

        # 비상 정지 버튼 (크게)
        tk.Button(self.root, text="EMERGENCY KILL", command=self.cmd_disarm, bg="red", fg="white", font=("Arial", 12, "bold"), height=2).pack(pady=10, fill="x", padx=15)

        # [4] 키보드 조종 가이드
        guide = tk.Label(self.root, text="[ Keyboard Control ]\nW/S: 전/후 | A/D: 좌/우\nUp/Down: 상승/하강 | Q/E: 회전", justify="center", fg="gray")
        guide.pack(pady=10)

        # 키보드 이벤트 바인딩
        self.root.bind('<w>', lambda e: self.update_target(dx=self.node.step_size))
        self.root.bind('<s>', lambda e: self.update_target(dx=-self.node.step_size))
        self.root.bind('<a>', lambda e: self.update_target(dy=-self.node.step_size))
        self.root.bind('<d>', lambda e: self.update_target(dy=self.node.step_size))
        self.root.bind('<Up>', lambda e: self.update_target(dz=-0.2))
        self.root.bind('<Down>', lambda e: self.update_target(dz=0.2))
        self.root.bind('<q>', lambda e: self.update_target(dyaw=-10.0))
        self.root.bind('<e>', lambda e: self.update_target(dyaw=10.0))

        self.refresh_ui()

    # 버튼 함수들
    def cmd_arm(self):
        self.node.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        print("Arming command sent")

    def cmd_disarm(self):
        self.node.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        print("Disarm command sent")

    def cmd_offboard(self):
        self.node.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        print("Offboard mode request sent")

    def cmd_land(self):
        # 4: Land mode (일부 설정에 따라 다를 수 있음)
        self.node.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        print("Landing command sent")

    def update_target(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        self.node.target_x += dx
        self.node.target_y += dy
        self.node.target_z += dz
        self.node.target_yaw += dyaw
        self.lbl_target.config(text=f"Target -> X: {self.node.target_x:.1f} | Y: {self.node.target_y:.1f} | Z: {self.node.target_z:.1f}")
        self.lbl_yaw.config(text=f"Target Yaw: {self.node.target_yaw:.1f}°")

    def refresh_ui(self):
        # 실시간 위치 표시
        self.lbl_pos.config(text=f"X: {self.node.curr_x:.2f} | Y: {self.node.curr_y:.2f} | Z: {self.node.curr_z:.2f}")
        
        # Arming 상태 표시 (2: Armed, 1: Disarmed)
        if self.node.arming_state == 2:
            self.lbl_arm.config(text="● ARMED", fg="green")
        else:
            self.lbl_arm.config(text="○ DISARMED", fg="red")

        self.root.after(100, self.refresh_ui)

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    node = DroneMasterController()
    # ROS 스핀을 별도 스레드에서 실행 (GUI 멈춤 방지)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    
    app = DroneGui(node)
    app.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()