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

        # 1. QoS 설정
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

        # 3. Subscriber 설정
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        # 4. 상태 변수
        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        self.arming_state = 0 
        self.nav_state = 0

        # 5. 목표 변수 (기본 이륙 고도 -2m)
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, -2.0
        self.target_yaw = 0.0
        self.step_size = 0.2 

        # 6. 주기적 명령 송신 타이머 (10Hz)
        self.create_timer(0.1, self.timer_callback)
        self.stop_offboard_control = False

    def pos_callback(self, msg):
        self.curr_x, self.curr_y, self.curr_z = msg.x, msg.y, msg.z
        self.curr_yaw = msg.heading

    def status_callback(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        if self.arming_state == 1: 
            self.stop_offboard_control = False
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = -2.0 
            self.target_yaw = 0.0

    def timer_callback(self):
        if self.nav_state == 4 or self.stop_offboard_control:
            return
        self.publish_offboard_mode()
        self.publish_setpoint()

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)]
        msg.yaw = self.target_yaw * (math.pi / 180.0)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, command, p1=0.0, p2=0.0):
        if command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.stop_offboard_control = True
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = command, p1, p2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

class DroneGui:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("PX4 Master GCS")
        self.root.geometry("450x850") # 입력창 추가를 위해 세로 길이 조정

        # [1] 드론 현재 상태 표시
        status_frame = tk.LabelFrame(self.root, text="Drone Status", padx=15, pady=10, fg="darkblue", font=("Arial", 10, "bold"))
        status_frame.pack(pady=5, fill="x", padx=10)
        self.lbl_pos = tk.Label(status_frame, text="X: 0.00 | Y: 0.00 | Z: 0.00", font=("Consolas", 11))
        self.lbl_pos.pack()
        self.lbl_arm = tk.Label(status_frame, text="DISARMED", font=("Arial", 12, "bold"), fg="red")
        self.lbl_arm.pack()

        # [2] 목표 좌표 현황 표시
        target_frame = tk.LabelFrame(self.root, text="Current Target", padx=15, pady=10)
        target_frame.pack(pady=5, fill="x", padx=10)
        self.lbl_target = tk.Label(target_frame, text="Target -> X: 0.0 | Y: 0.0 | Z: -2.0", font=("Arial", 10))
        self.lbl_target.pack()
        self.lbl_yaw = tk.Label(target_frame, text="Target Yaw: 0.0°", font=("Arial", 10))
        self.lbl_yaw.pack()

        # [3] 추가됨: 숫자 직접 입력 제어 (Coordinate Input)
        input_frame = tk.LabelFrame(self.root, text="Direct Coordinate Input", padx=15, pady=15, fg="darkgreen", font=("Arial", 10, "bold"))
        input_frame.pack(pady=10, fill="x", padx=10)

        # X, Y, Z, Yaw 입력창 레이아웃
        labels = ["X (North):", "Y (East):", "Z (Down):", "Yaw (Deg):"]
        self.entries = {}
        for i, label in enumerate(labels):
            tk.Label(input_frame, text=label).grid(row=i, column=0, sticky="e", pady=2)
            entry = tk.Entry(input_frame, width=10)
            entry.grid(row=i, column=1, pady=2, padx=5)
            # 초기값 설정
            if "Z" in label: entry.insert(0, "-2.0")
            else: entry.insert(0, "0.0")
            self.entries[label] = entry

        tk.Button(input_frame, text="Go to Coordinate", command=self.cmd_move_to, bg="#20B2AA", fg="white", font=("Arial", 10, "bold")).grid(row=0, column=2, rowspan=4, padx=10, sticky="nswe")

        # [4] 기본 명령 버튼
        cmd_frame = tk.LabelFrame(self.root, text="Basic Commands", padx=15, pady=10)
        cmd_frame.pack(pady=5, fill="x", padx=10)
        tk.Button(cmd_frame, text="1. ARM (시동)", command=self.cmd_arm, bg="#FF8C00", fg="white", height=2, width=15).grid(row=0, column=0, padx=5, pady=5)
        tk.Button(cmd_frame, text="2. OFFBOARD", command=self.cmd_offboard, bg="#4682B4", fg="white", height=2, width=15).grid(row=0, column=1, padx=5, pady=5)
        tk.Button(cmd_frame, text="DISARM (정지)", command=self.cmd_disarm, bg="#A9A9A9", height=2, width=15).grid(row=1, column=0, padx=5, pady=5)
        tk.Button(cmd_frame, text="LAND (착륙)", command=self.cmd_land, bg="#8FBC8F", height=2, width=15).grid(row=1, column=1, padx=5, pady=5)

        tk.Button(self.root, text="EMERGENCY KILL", command=self.cmd_disarm, bg="red", fg="white", font=("Arial", 12, "bold"), height=2).pack(pady=10, fill="x", padx=15)

        # [5] 키보드 조종 가이드 & 바인딩
        tk.Label(self.root, text="[ Keyboard ]\nW/S: 전/후 | A/D: 좌/우\nUp/Down: 상/하 | Q/E: 회전", justify="center", fg="gray").pack(pady=5)
        self.root.bind('<w>', lambda e: self.update_target(dx=self.node.step_size))
        self.root.bind('<s>', lambda e: self.update_target(dx=-self.node.step_size))
        self.root.bind('<a>', lambda e: self.update_target(dy=-self.node.step_size))
        self.root.bind('<d>', lambda e: self.update_target(dy=self.node.step_size))
        self.root.bind('<Up>', lambda e: self.update_target(dz=-0.2))
        self.root.bind('<Down>', lambda e: self.update_target(dz=0.2))
        self.root.bind('<q>', lambda e: self.update_target(dyaw=-10.0))
        self.root.bind('<e>', lambda e: self.update_target(dyaw=10.0))

        self.refresh_ui()

    def cmd_move_to(self):
        """ 입력된 숫자를 읽어 목표 좌표를 한 번에 업데이트 """
        try:
            self.node.target_x = float(self.entries["X (North):"].get())
            self.node.target_y = float(self.entries["Y (East):"].get())
            self.node.target_z = float(self.entries["Z (Down):"].get())
            self.node.target_yaw = float(self.entries["Yaw (Deg):"].get())
            self.update_ui_labels()
            print(f"Moving to: X={self.node.target_x}, Y={self.node.target_y}, Z={self.node.target_z}")
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers.")

    def cmd_arm(self): self.node.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    def cmd_disarm(self): self.node.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    def cmd_offboard(self): self.node.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    def cmd_land(self): self.node.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def update_target(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        self.node.target_x += dx
        self.node.target_y += dy
        self.node.target_z += dz
        self.node.target_yaw += dyaw
        self.update_ui_labels()

    def update_ui_labels(self):
        self.lbl_target.config(text=f"Target -> X: {self.node.target_x:.1f} | Y: {self.node.target_y:.1f} | Z: {self.node.target_z:.1f}")
        self.lbl_yaw.config(text=f"Target Yaw: {self.node.target_yaw:.1f}°")

    def refresh_ui(self):
        self.lbl_pos.config(text=f"X: {self.node.curr_x:.2f} | Y: {self.node.curr_y:.2f} | Z: {self.node.curr_z:.2f}")
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
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    app = DroneGui(node)
    app.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
