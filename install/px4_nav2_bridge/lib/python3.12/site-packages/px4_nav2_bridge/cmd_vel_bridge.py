#!/usr/bin/env python3
"""
cmd_vel -> PX4 Offboard 橋接節點.

功能：
  - 訂閱 Nav2 的 /cmd_vel（機體 FLU 座標），轉成 PX4 世界 NED 速度
  - 以 20 Hz 持續送 OffboardControlMode + TrajectorySetpoint（heartbeat）
  - 高度用位置鎖住（target_alt），xy 用速度、yaw 用角速度
  - cmd_vel 逾時 -> 鎖在當下位置懸停
  - 簡易 geofence（map 座標）：禁止往場地外的速度分量
  - 服務：~/start（切 Offboard + arm，會自動爬升到 target_alt）、~/land

使用：
  ros2 run px4_nav2_bridge cmd_vel_bridge --ros-args -p target_alt:=1.0
  ros2 service call /cmd_vel_bridge/start std_srvs/srv/Trigger
  ros2 service call /cmd_vel_bridge/land  std_srvs/srv/Trigger
"""
import math

from geometry_msgs.msg import Twist, TwistStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

NAN = float('nan')


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ---------- 參數 ----------
        # Jazzy 的 Nav2 輸出 TwistStamped；Humble 設 False
        self.declare_parameter('use_stamped', True)
        # PX4 topic 名稱（v1.16+ 有訊息版本化，部分 topic 可能帶 _v1 等後綴，請用 ros2 topic list 確認）
        self.declare_parameter('topic_offboard_mode', '/fmu/in/offboard_control_mode')
        self.declare_parameter('topic_setpoint', '/fmu/in/trajectory_setpoint')
        self.declare_parameter('topic_command', '/fmu/in/vehicle_command')
        self.declare_parameter('topic_odometry', '/fmu/out/vehicle_odometry')
        self.declare_parameter('target_alt', 1.0)         # 飛行高度 [m]，向上為正
        self.declare_parameter('max_xy_vel', 0.3)         # [m/s]
        self.declare_parameter('max_yaw_rate', 0.5)       # [rad/s]
        self.declare_parameter('cmd_timeout', 0.5)        # [s] 沒收到 cmd_vel 就懸停
        self.declare_parameter('odom_timeout', 0.3)       # [s] 沒收到 odometry 就懸停
        # geofence，map 座標 [m]（map = OptiTrack 世界座標：X 前、Y 左、Z 上）
        self.declare_parameter('fence_x_min', -2.0)
        self.declare_parameter('fence_x_max', 2.0)
        self.declare_parameter('fence_y_min', -2.0)
        self.declare_parameter('fence_y_max', 2.0)

        def p(n):
            return self.get_parameter(n).value

        self.target_alt = p('target_alt')
        self.max_xy = p('max_xy_vel')
        self.max_yaw_rate = p('max_yaw_rate')
        self.cmd_timeout = p('cmd_timeout')
        self.odom_timeout = p('odom_timeout')
        self.fence = (p('fence_x_min'), p('fence_x_max'), p('fence_y_min'), p('fence_y_max'))

        # ---------- QoS（PX4 uXRCE-DDS 需要 best effort） ----------
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------- Publishers ----------
        self.pub_mode = self.create_publisher(
            OffboardControlMode, p('topic_offboard_mode'), px4_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, p('topic_setpoint'), px4_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, p('topic_command'), px4_qos)

        # ---------- Subscribers ----------
        self.create_subscription(VehicleOdometry, p('topic_odometry'), self.on_odom, px4_qos)
        if p('use_stamped'):
            self.create_subscription(TwistStamped, '/cmd_vel', lambda m: self.on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        # ---------- Services ----------
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/land', self.srv_land)

        # ---------- 狀態 ----------
        self.cmd = Twist()
        self.last_cmd_t = None
        self.pos_ned = None          # [n, e, d]
        self.yaw_ned = 0.0
        self.last_odom_t = None
        self.hold_ne = None          # 懸停時鎖住的 [n, e]

        self.create_timer(0.05, self.loop)   # 20 Hz
        self.get_logger().info('cmd_vel bridge ready')

    # ---------- callbacks ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, twist: Twist):
        self.cmd = twist
        self.last_cmd_t = self.now_s()

    def on_odom(self, msg: VehicleOdometry):
        self.pos_ned = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        w, x, y, z = msg.q  # PX4: q = [w, x, y, z]，FRD -> NED
        self.yaw_ned = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.last_odom_t = self.now_s()

    # ---------- 主迴圈 ----------
    def loop(self):
        t = self.now_s()
        cmd_ok = self.last_cmd_t is not None and (t - self.last_cmd_t) < self.cmd_timeout
        odom_ok = self.last_odom_t is not None and (t - self.last_odom_t) < self.odom_timeout

        sp = TrajectorySetpoint()
        sp.timestamp = self.px4_ts()
        z_ned = -self.target_alt

        if cmd_ok and odom_ok:
            self.hold_ne = None
            vn, ve, yaw_rate = self.convert(self.cmd)
            vn, ve = self.apply_fence(vn, ve)
            sp.position = [NAN, NAN, z_ned]
            sp.velocity = [vn, ve, NAN]
            sp.yaw = NAN
            sp.yawspeed = yaw_rate
        else:
            # 懸停：鎖住進入懸停瞬間的位置
            if self.hold_ne is None and self.pos_ned is not None:
                self.hold_ne = self.pos_ned[:2]
                if not odom_ok:
                    self.get_logger().warn('odometry 逾時，懸停')
            if self.hold_ne is not None:
                sp.position = [self.hold_ne[0], self.hold_ne[1], z_ned]
            else:
                sp.position = [NAN, NAN, z_ned]
            sp.velocity = [0.0, 0.0, NAN] if self.hold_ne is None else [NAN, NAN, NAN]
            sp.yaw = self.yaw_ned
            sp.yawspeed = NAN

        mode = OffboardControlMode()
        mode.timestamp = sp.timestamp
        mode.position = True
        mode.velocity = True
        self.pub_mode.publish(mode)
        self.pub_sp.publish(sp)

    # ---------- 座標轉換 ----------
    def convert(self, tw: Twist):
        # FLU -> FRD
        vx_b = tw.linear.x
        vy_b = -tw.linear.y
        # 限速
        n = math.hypot(vx_b, vy_b)
        if n > self.max_xy:
            vx_b, vy_b = vx_b * self.max_xy / n, vy_b * self.max_xy / n
        # 機體 FRD -> 世界 NED（用 yaw 旋轉）
        c, s = math.cos(self.yaw_ned), math.sin(self.yaw_ned)
        vn = c * vx_b - s * vy_b
        ve = s * vx_b + c * vy_b
        # FLU 的逆時針為正 -> FRD/NED 的順時針為正
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, -tw.angular.z))
        return vn, ve, yaw_rate

    def apply_fence(self, vn, ve):
        if self.pos_ned is None:
            return 0.0, 0.0
        # PX4 local(NED) -> map(OptiTrack)：與 mocap_px4_bridge 相同慣例 x = n, y = -e
        x, y = self.pos_ned[0], -self.pos_ned[1]
        vx, vy = vn, -ve
        x_min, x_max, y_min, y_max = self.fence
        if (x >= x_max and vx > 0) or (x <= x_min and vx < 0):
            vx = 0.0
        if (y >= y_max and vy > 0) or (y <= y_min and vy < 0):
            vy = 0.0
        return vx, -vy   # 回到 (vn, ve)

    # ---------- 指令 ----------
    def px4_ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.px4_ts()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        # target_system=0 是廣播：不同載具的 MAV_SYS_ID 不一定是 1（例如這台 MAV4
        # 實際是 4），寫死成特定 ID 會被 Commander 的 target_system 檢查直接忽略。
        # 每個 UAV 走自己獨立的 /<namespace>/fmu/in/vehicle_command topic，所以廣播
        # 不會誤發給別的機隻。
        m.target_system = 0
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def srv_start(self, req, res):
        if self.last_odom_t is None:
            res.success, res.message = False, '還沒收到 odometry'
            return res
        # heartbeat 一直在送，直接切 Offboard 再 arm；setpoint 的 z 會讓它爬升到 target_alt
        self.hold_ne = self.pos_ned[:2]
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        res.success, res.message = True, 'Offboard + arm 已送出'
        return res

    def srv_land(self, req, res):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        res.success, res.message = True, 'Land 已送出'
        return res


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
