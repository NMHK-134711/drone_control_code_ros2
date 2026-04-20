"""
vision_area_painter_node.py
===========================
4포인트 영역 채우기 + Vision 장애물 회피 통합 노드

역할:
  1. 4개 꼭짓점으로 도색 영역 자동 계산 (area_painter_node)
  2. 비행 중 Vision으로 window/blind/balcony 감지 (vision_painter_node)
  3. 감지된 장애물은 해당 줄에서 자동 스킵 or 우회

흐름:
  [사전]
  Vision → facade BBox 4꼭짓점 추출
  → /painting/start_area 로 전송

  [실행 중]
  지그재그 비행
  + /vision/target_error 구독
  → window/blind 감지 시
     ① 해당 Y 구간 스킵 (장애물 너비만큼)
     ② 또는 Z축 한 줄 건너뜀

Subscribe:
  /fmu/out/vehicle_local_position
  /fmu/out/vehicle_status
  /painting/start_area          영역 지정 (JSON String)
  /vision/target_error          Vision BBox 오차 (Point)
  /vision/bboxes_2d             BBox 상세 정보 (Detection2DArray)

Publish:
  /fmu/in/offboard_control_mode
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command
  /painting/status              현재 상태
  /painting/waypoints_count     진행도

명령 형식 (/painting/start_area):
  {
    "points": [
      {"y": -1.5, "z": -1.0},   // P0 좌상단 (낮은고도)
      {"y":  1.5, "z": -1.0},   // P1 우상단
      {"y":  1.5, "z": -3.0},   // P2 우하단 (높은고도)
      {"y": -1.5, "z": -3.0}    // P3 좌하단
    ],
    "wall_x": 2.5,
    "step": 0.4
    
    ros2 topic pub --once /painting/start_area std_msgs/String \
  'data: "{\"points\":[{\"y\":-1.5,\"z\":-1.0},{\"y\":1.5,\"z\":-1.0},{\"y\":1.5,\"z\":-3.0},{\"y\":-1.5,\"z\":-3.0}],\"wall_x\":2.5,\"step\":0.4}"'
  }

NED 좌표계:
  X = North (벽 방향, 고정)
  Y = East  (좌우)
  Z = Down  (고도, 음수가 위)

회피 동작:
  Vision err_x (좌우 오차) 기준:
    │err_x│ > avoid_threshold → 장애물 감지
    → 현재 Y 위치에서 장애물 너비만큼 스킵
    → 다음 안전 웨이포인트로 점프

실행:
  ros2 run px4_gui_ctrl vision_area_painter_node --ros-args \
      -p avoid_threshold:=0.35 \
      -p vision_gain_y:=0.3
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math
import json
import numpy as np
from enum import Enum, auto
from dataclasses import dataclass
from typing import List

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from geometry_msgs.msg import Point
from std_msgs.msg import String

try:
    from vision_msgs.msg import Detection2DArray
    VISION_MSGS_AVAILABLE = True
except ImportError:
    VISION_MSGS_AVAILABLE = False


# ==================== 상태 머신 ====================

class State(Enum):
    IDLE          = auto()
    ARMING        = auto()
    TAKEOFF       = auto()
    MOVE_TO_START = auto()
    PAINTING      = auto()   # 정상 지그재그 이동
    AVOIDING      = auto()   # 장애물 감지 → 우회 중
    RETURN_HOME   = auto()
    LANDING       = auto()
    DONE          = auto()


# ==================== 웨이포인트 ====================

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    skip: bool = False   # True면 이 줄은 장애물로 인해 스킵

    def to_list(self):
        return [self.x, self.y, self.z]


# ==================== 영역 → 웨이포인트 생성 ====================

def generate_area_waypoints(
    p0: dict, p1: dict, p2: dict, p3: dict,
    wall_x: float,
    step: float = 0.4,
) -> List[Waypoint]:
    """
    4개 꼭짓점 → 지그재그 웨이포인트 생성

    P0(y_min, z_max) ─ P1(y_max, z_max)  ← 낮은 고도 (z_max, 덜 음수)
         │                    │
    P3(y_min, z_min) ─ P2(y_max, z_min)  ← 높은 고도 (z_min, 더 음수)

    z_max(낮은 고도)에서 z_min(높은 고도)으로 step씩 상승
    각 줄: Y축 좌우 교대
    """
    all_y = [p['y'] for p in [p0, p1, p2, p3]]
    all_z = [p['z'] for p in [p0, p1, p2, p3]]

    y_min = min(all_y)
    y_max = max(all_y)
    z_min = min(all_z)   # 더 음수 = 더 높은 고도
    z_max = max(all_z)   # 덜 음수 = 더 낮은 고도

    if (y_max - y_min) < 0.1 or abs(z_max - z_min) < 0.1:
        raise ValueError(
            f'영역이 너무 작음: Y={y_max-y_min:.2f}m, Z={abs(z_max-z_min):.2f}m'
        )

    waypoints = []
    current_z = z_max
    direction = 1   # 1: y_min→y_max, -1: y_max→y_min

    while current_z >= z_min:
        if direction == 1:
            waypoints.append(Waypoint(wall_x, y_min, round(current_z, 3)))
            waypoints.append(Waypoint(wall_x, y_max, round(current_z, 3)))
        else:
            waypoints.append(Waypoint(wall_x, y_max, round(current_z, 3)))
            waypoints.append(Waypoint(wall_x, y_min, round(current_z, 3)))

        current_z -= step
        direction *= -1

    return waypoints


# ==================== Node ====================

class VisionAreaPainterNode(Node):
    """
    4포인트 영역 채우기 + Vision 장애물 회피 통합 노드
    """

    def __init__(self):
        super().__init__('vision_area_painter_node')

        # ---- 파라미터 ----
        self.declare_parameter('takeoff_alt',      -2.0)
        self.declare_parameter('wp_tolerance',      0.25)
        self.declare_parameter('default_wall_x',    2.5)
        self.declare_parameter('default_step',      0.4)
        # Vision IBVS 관련
        self.declare_parameter('vision_gain_y',     0.3)
        self.declare_parameter('vision_gain_z',     0.2)
        self.declare_parameter('max_correction',    0.4)
        self.declare_parameter('avoid_threshold',   0.35)
        self.declare_parameter('avoid_skip_count',  2)
        self.declare_parameter('vision_timeout',    1.0)
        # SLAM Odometry 벽 거리 유지 관련 ← NEW
        self.declare_parameter('wall_x_gain',       0.5)    # X축 거리 보정 게인
        self.declare_parameter('wall_x_tolerance',  0.1)    # 허용 오차 (m)
        self.declare_parameter('wall_x_max_corr',   0.3)    # 최대 X 보정량 (m)

        self.takeoff_alt      = self.get_parameter('takeoff_alt').value
        self.wp_tolerance     = self.get_parameter('wp_tolerance').value
        self.default_wall_x   = self.get_parameter('default_wall_x').value
        self.default_step     = self.get_parameter('default_step').value
        self.vision_gain_y    = self.get_parameter('vision_gain_y').value
        self.vision_gain_z    = self.get_parameter('vision_gain_z').value
        self.max_correction   = self.get_parameter('max_correction').value
        self.avoid_threshold  = self.get_parameter('avoid_threshold').value
        self.avoid_skip_count = self.get_parameter('avoid_skip_count').value
        self.vision_timeout   = self.get_parameter('vision_timeout').value
        self.wall_x_gain      = self.get_parameter('wall_x_gain').value
        self.wall_x_tolerance = self.get_parameter('wall_x_tolerance').value
        self.wall_x_max_corr  = self.get_parameter('wall_x_max_corr').value

        # ---- 상태 변수 ----
        self.state            = State.IDLE
        self.current_pos      = [0.0, 0.0, 0.0]
        self.current_yaw      = 0.0
        self.arming_state     = 0
        self.nav_state        = 0
        self.px4_timestamp    = 0

        # 웨이포인트
        self.waypoints: List[Waypoint] = []
        self.wp_index         = 0
        self.target           = [0.0, 0.0, self.takeoff_alt]
        self.corrected_target = [0.0, 0.0, self.takeoff_alt]

        # Vision
        self.vision_err_x     = 0.0
        self.vision_err_y     = 0.0
        self.vision_area      = 0.0
        self.vision_active    = False
        self.last_vision_time = None

        # SLAM Odometry 벽 거리 추적 ← NEW
        # current_pos[0] (X) 와 wall_x 비교 → X축 보정
        self.slam_x_correction = 0.0   # SLAM이 계산한 X 보정량

        # 회피 상태
        self.avoid_counter    = 0   # 연속 장애물 감지 횟수
        self.AVOID_CONFIRM    = 3   # N회 연속 감지 시 회피 확정 (오탐 방지)

        # Offboard 카운터
        self.offboard_counter = 0
        self.OFFBOARD_READY   = 10

        # 영역 경계 (회피 후 복귀 시 경계 준수용)
        self.y_min = 0.0
        self.y_max = 0.0
        self.z_min = 0.0
        self.z_max = 0.0
        self.wall_x = self.default_wall_x

        # ---- QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Subscribe ----
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_callback, px4_qos,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback, px4_qos,
        )
        self.area_sub = self.create_subscription(
            String,
            '/painting/start_area',
            self.area_callback, 10,
        )
        self.vision_sub = self.create_subscription(
            Point,
            '/vision/target_error',
            self.vision_callback, 10,
        )

        # Detection2DArray (BBox 상세 - 있으면 사용)
        if VISION_MSGS_AVAILABLE:
            self.bbox_sub = self.create_subscription(
                Detection2DArray,
                '/vision/bboxes_2d',
                self.bbox_callback, 10,
            )

        # ---- Publish ----
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',  px4_qos)
        self.command_pub  = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command',       px4_qos)
        self.status_pub   = self.create_publisher(
            String, '/painting/status', 10)
        self.wp_pub       = self.create_publisher(
            String, '/painting/waypoints_count', 10)

        # ---- 10Hz 타이머 ----
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'VisionAreaPainterNode 시작\n'
            '  /painting/start_area 로 영역 지정 대기 중\n'
            f'  회피 임계값: {self.avoid_threshold} | '
            f'Vision 게인 Y:{self.vision_gain_y} Z:{self.vision_gain_z}'
        )

    # ==================== Callbacks ====================

    def pos_callback(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]
        self.current_yaw = msg.heading

    def status_callback(self, msg):
        self.arming_state  = msg.arming_state
        self.nav_state     = msg.nav_state
        self.px4_timestamp = msg.timestamp

    def area_callback(self, msg: String):
        """4포인트 영역 수신 → 웨이포인트 생성 → 시작"""
        if self.state not in (State.IDLE, State.DONE):
            self.get_logger().warn(
                f'현재 {self.state.name} 중 - 완료 후 재시도')
            return

        try:
            data   = json.loads(msg.data)
            points = data['points']
            self.wall_x  = data.get('wall_x', self.default_wall_x)
            step         = data.get('step',   self.default_step)

            if len(points) != 4:
                raise ValueError(f'포인트 4개 필요 (받음: {len(points)}개)')

            p0, p1, p2, p3 = points

            # 영역 경계 저장 (회피 시 경계 준수용)
            all_y = [p['y'] for p in points]
            all_z = [p['z'] for p in points]
            self.y_min = min(all_y)
            self.y_max = max(all_y)
            self.z_min = min(all_z)
            self.z_max = max(all_z)

            # 웨이포인트 생성
            self.waypoints = generate_area_waypoints(
                p0, p1, p2, p3, self.wall_x, step)
            self.wp_index        = 0
            self.offboard_counter = 0
            self.state           = State.ARMING

            self.get_logger().info(
                f'✅ 영역 수신!\n'
                f'  Y: {self.y_min:.2f} ~ {self.y_max:.2f}m\n'
                f'  Z: {self.z_min:.2f} ~ {self.z_max:.2f}m\n'
                f'  X(벽): {self.wall_x:.2f}m | 간격: {step:.2f}m\n'
                f'  웨이포인트: {len(self.waypoints)}개\n'
                f'  예상거리: {self._estimate_total_dist():.1f}m'
            )

        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'영역 파싱 오류: {e}')

    def vision_callback(self, msg: Point):
        """
        Vision BBox 오차 수신
          msg.x = window 좌우 오차 [-1, 1]
          msg.y = window 상하 오차 [-1, 1]
          msg.z = BBox 면적 (거리 proxy)
        """
        self.vision_err_x     = msg.x
        self.vision_err_y     = msg.y
        self.vision_area      = msg.z
        self.last_vision_time = self.get_clock().now()
        self.vision_active    = True

    def bbox_callback(self, msg):
        """BBox 상세 정보 (현재는 로깅용, 나중에 더 정밀한 회피에 활용)"""
        obstacle_classes = ['window', 'balcony', 'blind']
        obstacles = [
            d for d in msg.detections
            if d.results and d.results[0].hypothesis.class_id in obstacle_classes
        ]
        if obstacles and self.state == State.PAINTING:
            self.get_logger().debug(
                f'장애물 감지: '
                f'{[d.results[0].hypothesis.class_id for d in obstacles]}',
                throttle_duration_sec=1.0,
            )

    # ==================== 메인 타이머 ====================

    def timer_callback(self):
        # Vision 타임아웃
        if self.last_vision_time is not None:
            dt = (self.get_clock().now() - self.last_vision_time).nanoseconds / 1e9
            if dt > self.vision_timeout:
                self.vision_active = False

        self._publish_offboard_mode()
        self._run_state_machine()
        self._publish_setpoint()
        self._publish_status()

    # ==================== 상태 머신 ====================

    def _run_state_machine(self):

        # ---- IDLE ----
        if self.state == State.IDLE:
            self.target = [0.0, 0.0, self.takeoff_alt]

        # ---- ARMING ----
        elif self.state == State.ARMING:
            self.target = [0.0, 0.0, self.takeoff_alt]
            self.offboard_counter += 1
            if self.offboard_counter >= self.OFFBOARD_READY:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = State.TAKEOFF
                self.get_logger().info('→ TAKEOFF')

        # ---- TAKEOFF ----
        elif self.state == State.TAKEOFF:
            self.target = [0.0, 0.0, self.takeoff_alt]
            if self._reached(self.target, tol=0.3):
                self.state = State.MOVE_TO_START
                self.get_logger().info('→ MOVE_TO_START')

        # ---- MOVE_TO_START ----
        elif self.state == State.MOVE_TO_START:
            if not self.waypoints:
                self.state = State.IDLE
                return
            self.target = self.waypoints[0].to_list()
            if self._reached(self.target, tol=0.3):
                self.state = State.PAINTING
                self.get_logger().info(
                    f'→ PAINTING | 총 {len(self.waypoints)}개 웨이포인트')

        # ---- PAINTING: 지그재그 + Vision 보정 ----
        elif self.state == State.PAINTING:
            if self.wp_index >= len(self.waypoints):
                self.state = State.RETURN_HOME
                self.get_logger().info('🎨 도색 완료 → RETURN_HOME')
                return

            wp = self.waypoints[self.wp_index]
            self.target = wp.to_list()

            # Vision 보정 적용
            self._apply_vision_correction()

            # 장애물 감지 → 회피 판단
            if self._detect_obstacle():
                self.avoid_counter += 1
                if self.avoid_counter >= self.AVOID_CONFIRM:
                    self._trigger_avoid()
                    return
            else:
                self.avoid_counter = 0

            # 웨이포인트 도달 체크
            if self._reached(self.corrected_target):
                self.get_logger().info(
                    f'✅ WP {self.wp_index + 1}/{len(self.waypoints)} | '
                    f'Y={wp.y:.2f} Z={wp.z:.2f}'
                )
                self.wp_index += 1

        # ---- AVOIDING: 장애물 우회 ----
        elif self.state == State.AVOIDING:
            """
            회피 전략:
              장애물(window/blind)이 감지된 Y 구간을 스킵
              → avoid_skip_count개 웨이포인트 건너뜀
              → 장애물 너비만큼 자동으로 피해감

            예시:
              WP5(y=-1.5) → WP6(y=1.5) 이동 중 window 감지
              → WP6 스킵 → WP7(y=-1.5, z=-1.4)로 점프
              → 해당 Y 구간 페인팅 없이 다음 줄로
            """
            # 즉시 다음 안전 웨이포인트로 점프
            skip_target = min(
                self.wp_index + self.avoid_skip_count,
                len(self.waypoints) - 1
            )
            self.wp_index = skip_target
            self.avoid_counter = 0
            self.state = State.PAINTING
            self.get_logger().info(
                f'↩️  회피 완료 → WP {self.wp_index + 1}번으로 점프'
            )

        # ---- RETURN_HOME ----
        elif self.state == State.RETURN_HOME:
            self.target           = [0.0, 0.0, self.takeoff_alt]
            self.corrected_target = list(self.target)
            if self._reached(self.target, tol=0.3):
                self.state = State.LANDING
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().info('→ LANDING')

        # ---- LANDING ----
        elif self.state == State.LANDING:
            if abs(self.current_pos[2]) < 0.3 or self.arming_state == 1:
                self.state = State.DONE
                self.get_logger().info('🎉 전체 완료!')

    # ==================== Vision 보정 (IBVS + SLAM) ====================

    def _apply_vision_correction(self):
        """
        IBVS + SLAM Odometry 융합 보정

        역할 분담:
          IBVS (Vision):
            err_x → Y축 보정 (좌우)   ← 창문이 화면 어디 있는지
            err_y → Z축 보정 (상하)   ← 창문이 화면 어디 있는지

          SLAM Odometry:
            current_pos[0] vs wall_x → X축 보정 (벽 거리 유지)
            "지금 내가 벽에서 얼마나 떨어져 있나"

        예시:
          목표 wall_x = 2.5m
          현재 X = 2.8m (벽에서 너무 멀어짐)
          → X 보정 = +0.3m (앞으로 이동)

          창문이 화면 오른쪽으로 치우침 (err_x = +0.4)
          → Y 보정 = +0.12m (오른쪽 이동)
        """

        # ---- X축: SLAM Odometry 기반 벽 거리 유지 ----
        # current_pos[0] = 현재 드론 X위치 (VehicleLocalPosition에서)
        # wall_x         = 목표 벽 거리
        x_error = self.wall_x - self.current_pos[0]   # 양수: 앞으로 가야 함

        if abs(x_error) > self.wall_x_tolerance:
            # 허용 오차 벗어남 → 보정
            self.slam_x_correction = float(np.clip(
                x_error * self.wall_x_gain,
                -self.wall_x_max_corr,
                self.wall_x_max_corr,
            ))
        else:
            # 허용 오차 내 → 보정 없음
            self.slam_x_correction = 0.0

        # ---- Y/Z축: IBVS Vision 오차 기반 보정 ----
        if not self.vision_active:
            # Vision 신호 없음 → Y/Z 보정 없이 SLAM X만 적용
            self.corrected_target = [
                self.wall_x + self.slam_x_correction,
                self.target[1],
                self.target[2],
            ]
            return

        corr_y = float(np.clip(
            self.vision_err_x * self.vision_gain_y,
            -self.max_correction, self.max_correction,
        ))
        corr_z = float(np.clip(
            self.vision_err_y * self.vision_gain_z,
            -self.max_correction, self.max_correction,
        ))

        # 영역 경계 클리핑 (4포인트 영역 절대 벗어나지 않음)
        corrected_y = float(np.clip(
            self.target[1] + corr_y, self.y_min, self.y_max))
        corrected_z = float(np.clip(
            self.target[2] + corr_z, self.z_min, self.z_max))

        # X = SLAM 거리 유지, Y/Z = IBVS 보정
        self.corrected_target = [
            self.wall_x + self.slam_x_correction,
            corrected_y,
            corrected_z,
        ]

        self.get_logger().debug(
            f'IBVS+SLAM | '
            f'X오차:{self.wall_x - self.current_pos[0]:+.2f}m → X보정:{self.slam_x_correction:+.3f}m | '
            f'Vision err:({self.vision_err_x:.2f},{self.vision_err_y:.2f}) | '
            f'Y보정:{corr_y:+.3f}m Z보정:{corr_z:+.3f}m',
        )

    def _detect_obstacle(self) -> bool:
        """
        장애물 감지 판단
        Vision err_x가 avoid_threshold 초과 = window/blind가 경로를 막고 있음
        """
        if not self.vision_active:
            return False
        return abs(self.vision_err_x) > self.avoid_threshold

    def _trigger_avoid(self):
        """회피 트리거"""
        self.state = State.AVOIDING
        self.get_logger().warn(
            f'⚠️  장애물 감지! err_x={self.vision_err_x:.2f} > {self.avoid_threshold}\n'
            f'   WP {self.wp_index + 1} 스킵 → {self.avoid_skip_count}개 건너뜀'
        )

    # ==================== PX4 발행 ====================

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self):
        # PAINTING/AVOIDING은 보정된 목표, 나머지는 원본
        if self.state in (State.PAINTING,):
            t = self.corrected_target
        else:
            t = self.target

        msg = TrajectorySetpoint()
        msg.position     = [float(t[0]), float(t[1]), float(t[2])]
        msg.yaw          = 0.0   # 항상 벽(X+) 방향 고정
        msg.velocity     = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk         = [float('nan')] * 3
        msg.yawspeed     = float('nan')
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 255
        msg.source_component = 191
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    # ==================== 유틸 ====================

    def _reached(self, target, tol=None):
        tol = tol or self.wp_tolerance
        dist = math.sqrt(sum(
            (self.current_pos[i] - target[i]) ** 2 for i in range(3)
        ))
        return dist < tol

    def _estimate_total_dist(self) -> float:
        if len(self.waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.waypoints)):
            p1 = self.waypoints[i-1].to_list()
            p2 = self.waypoints[i].to_list()
            total += math.sqrt(sum((a-b)**2 for a, b in zip(p1, p2)))
        return total

    def _publish_status(self):
        vision_str = (
            f'IBVS:({self.vision_err_x:.2f},{self.vision_err_y:.2f})'
            if self.vision_active else 'vision=없음'
        )
        slam_str = f'SLAM_X보정:{self.slam_x_correction:+.3f}m'
        avoid_str = (
            f' | 회피감지:{self.avoid_counter}/{self.AVOID_CONFIRM}'
            if self.state == State.PAINTING and self.avoid_counter > 0 else ''
        )

        status = String()
        status.data = (
            f'[{self.state.name}] '
            f'WP:{self.wp_index}/{len(self.waypoints)} | '
            f'pos:({self.current_pos[0]:.2f},'
            f'{self.current_pos[1]:.2f},'
            f'{self.current_pos[2]:.2f}) | '
            f'{vision_str} | {slam_str}{avoid_str}'
        )
        self.status_pub.publish(status)

        wp_msg = String()
        wp_msg.data = f'{self.wp_index}/{len(self.waypoints)}'
        self.wp_pub.publish(wp_msg)


# ==================== Main ====================

def main(args=None):
    rclpy.init(args=args)
    node = VisionAreaPainterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료')
        node._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
