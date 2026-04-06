import sys
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 메시지 임포트
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
import tkinter as tk
from tkinter import messagebox

class PX4GuiControl(Node):
    def __init__(self):
        super().__init__('px4_gui_control')

        # 1. QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Publisher & Subscriber 설정
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # 현재 위치 및 방향 구독
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            qos_profile
        )

        # 3. 상태 변수 (현재 위치/방향)
        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        
        # 4. 목표 변수 (제어용)
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_yaw = 0.0 # Degree 단위로 입력받아 Radian으로 변환 예정

        # 5. 주기적 Heartbeat 송신 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        """ 드론으로부터 실시간 위치 데이터를 받는 콜백 """
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z
        # PX4의 heading은 Radian이며, NED 기준이므로 시각화를 위해 가공 가능
        self.curr_yaw = msg.heading 

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.acceleration = True, False, False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)]
        # 입력받은 Degree를 Radian으로 변환하여 송신
        msg.yaw = self.target_yaw * (math.pi / 180.0)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1, msg.param2 = param1, param2
        msg.command = command
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

# --- GUI 클래스 ---
class DroneApp:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("PX4 Advanced Controller")
        self.root.geometry("400x600")

        # 실시간 상태 표시창
        status_frame = tk.LabelFrame(self.root, text="Current Status (NED)", padx=10, pady=10)
        status_frame.pack(pady=10, fill="x")

        self.lbl_pos = tk.Label(status_frame, text="X: 0.0 | Y: 0.0 | Z: 0.0", font=("Arial", 10, "bold"))
        self.lbl_pos.pack()
        self.lbl_yaw = tk.Label(status_frame, text="Yaw: 0.0°", font=("Arial", 10, "bold"))
        self.lbl_yaw.pack()

        # 제어 버튼
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="ARM", command=self.node.arm, bg="orange", width=10).grid(row=0, column=0, padx=5)
        tk.Button(btn_frame, text="OFFBOARD", command=self.node.set_offboard, bg="lightblue", width=10).grid(row=0, column=1, padx=5)
        tk.Button(self.root, text="EMERGENCY DISARM", command=self.node.disarm, bg="red", fg="white", width=25).pack(pady=5)

        # 좌표 및 방향 입력
        input_frame = tk.LabelFrame(self.root, text="Target Setpoints", padx=10, pady=10)
        input_frame.pack(pady=10, fill="x")

        tk.Label(input_frame, text="North (X):").grid(row=0, column=0)
        self.ent_x = tk.Entry(input_frame); self.ent_x.insert(0, "0.0"); self.ent_x.grid(row=0, column=1)

        tk.Label(input_frame, text="East (Y):").grid(row=1, column=0)
        self.ent_y = tk.Entry(input_frame); self.ent_y.insert(0, "0.0"); self.ent_y.grid(row=1, column=1)

        tk.Label(input_frame, text="Down (Z):").grid(row=2, column=0)
        self.ent_z = tk.Entry(input_frame); self.ent_z.insert(0, "-5.0"); self.ent_z.grid(row=2, column=1)

        tk.Label(input_frame, text="Yaw (Deg):").grid(row=3, column=0)
        self.ent_yaw = tk.Entry(input_frame); self.ent_yaw.insert(0, "0.0"); self.ent_yaw.grid(row=3, column=1)

        tk.Button(self.root, text="UPDATE TARGET", command=self.update_target, bg="green", fg="white", height=2).pack(pady=10)

        # GUI 갱신 루프
        self.update_gui()

    def update_target(self):
        try:
            self.node.target_x = float(self.ent_x.get())
            self.node.target_y = float(self.ent_y.get())
            self.node.target_z = float(self.ent_z.get())
            self.node.target_yaw = float(self.ent_yaw.get())
        except ValueError:
            messagebox.showerror("Error", "숫자만 입력해주세요.")

    def update_gui(self):
        # ROS 노드에서 받은 데이터를 GUI 레이블에 업데이트
        self.lbl_pos.config(text=f"X: {self.node.curr_x:.2f} | Y: {self.node.curr_y:.2f} | Z: {self.node.curr_z:.2f}")
        # Radian을 다시 Degree로 바꿔서 표시
        curr_yaw_deg = self.node.curr_yaw * (180.0 / math.pi)
        self.lbl_yaw.config(text=f"Yaw: {curr_yaw_deg:.1f}°")
        
        # 100ms마다 GUI 갱신 (0.1초)
        self.root.after(100, self.update_gui)

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    node = PX4GuiControl()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = DroneApp(node)
    app.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
