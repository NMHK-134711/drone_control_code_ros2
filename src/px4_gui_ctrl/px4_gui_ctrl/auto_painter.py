import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
import math

class AutoPainterNode(Node):
    def __init__(self):
        super().__init__('auto_painter_node')

        # 퍼블리셔 (명령 하달)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 서브스크라이버 (현재 위치 파악)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, 10)

        # 10Hz 타이머 (오프보드 하트비트)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 현재 드론 위치
        self.current_pos = [0.0, 0.0, 0.0]

        # 도색 궤적(웨이포인트) 생성!
        self.waypoints = self.generate_paint_path(
            wall_distance=2.5,   # 드론과 벽 사이의 거리 (X축, 2.5m 앞)
            y_start=-1.5,        # 도색 시작점 (왼쪽 1.5m)
            y_end=1.5,           # 도색 끝점 (오른쪽 1.5m)
            z_bottom=-1.0,       # 도색 하단 높이 (1m 고도, NED 좌표계라 -)
            z_top=-3.0,          # 도색 상단 높이 (3m 고도, NED 좌표계라 -)
            z_step=-0.5          # 한 번 왕복 후 올라갈 높이 (0.5m씩 상승)
        )
        self.current_wp_index = 0
        self.is_painting = False

        self.get_logger().info("🎨 자동 도색 궤적 준비 완료! 이륙 명령을 대기합니다.")

    def generate_paint_path(self, wall_distance, y_start, y_end, z_bottom, z_top, z_step):
        """ ㄹ자(지그재그) 비행 궤적을 계산하여 리스트로 반환 """
        waypoints = []
        current_z = z_bottom
        direction = 1 # 1: 왼쪽->오른쪽, -1: 오른쪽->왼쪽

        # 이륙 후 벽 앞의 첫 시작점 대기 위치
        waypoints.append([wall_distance, y_start, current_z])

        while current_z >= z_top: # Z는 위로 갈수록 음수이므로 >= 사용
            if direction == 1:
                waypoints.append([wall_distance, y_start, current_z])
                waypoints.append([wall_distance, y_end, current_z])
            else:
                waypoints.append([wall_distance, y_end, current_z])
                waypoints.append([wall_distance, y_start, current_z])
            
            current_z += z_step # Z축 위로 상승 (z_step이 음수임)
            direction *= -1     # 방향 전환
        
        # 도색이 끝나면 최초 위치로 복귀
        waypoints.append([0.0, 0.0, -1.0]) 
        return waypoints

    def odom_callback(self, msg):
        """ 드론의 현재 위치를 지속적으로 업데이트 """
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]
        
        # 도색 중이고, 아직 갈 길이 남았다면
        if self.is_painting and self.current_wp_index < len(self.waypoints):
            target = self.waypoints[self.current_wp_index]
            
            # 현재 위치와 목표 웨이포인트 사이의 거리 계산 (유클리디안 거리)
            dist = math.sqrt(
                (self.current_pos[0] - target[0])**2 +
                (self.current_pos[1] - target[1])**2 +
                (self.current_pos[2] - target[2])**2
            )
            
            # 오차 0.2m 이내로 도달했으면 다음 목표로 인덱스 이동!
            if dist < 0.2:
                self.get_logger().info(f"✅ 웨이포인트 {self.current_wp_index} 도달! 다음 구역으로 이동합니다.")
                self.current_wp_index += 1

                if self.current_wp_index >= len(self.waypoints):
                    self.get_logger().info("🎉 도색 작업이 모두 완료되었습니다!")
                    self.is_painting = False

    def timer_callback(self):
        """ 10Hz로 목표 좌표 쏴주기 """
        # 오프보드 모드 하트비트
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

        # 도색 중이라면 궤적을 쏘고, 아니면 기본 이륙 위치 유지
        setpoint = TrajectorySetpoint()
        if self.is_painting and self.current_wp_index < len(self.waypoints):
            target = self.waypoints[self.current_wp_index]
            setpoint.position = [float(target[0]), float(target[1]), float(target[2])]
            # 드론의 머리(Yaw)는 항상 벽(X축 양의 방향, 0.0 라디안)을 바라보게 고정!
            setpoint.yaw = 0.0 
        else:
            # 대기 상태 (고도 1m)
            setpoint.position = [0.0, 0.0, -1.0]
            setpoint.yaw = 0.0

        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(setpoint)

    # (이륙, 오프보드 전환 등의 VehicleCommand 함수는 기존 GUI 노드와 동일하여 생략)