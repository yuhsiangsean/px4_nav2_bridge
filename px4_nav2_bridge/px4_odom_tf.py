#!/usr/bin/env python3
"""
PX4 vehicle_odometry -> Nav2 需要的 TF(odom -> base_link) 與 /odom.

- 只取平面資訊（x, y, yaw），高度照抄，roll/pitch 忽略（Nav2 是 2D）
- 座標慣例與 mocap_px4_bridge 一致，讓 map = OptiTrack 世界座標（X 前、Y 左、Z 上）：
    mocap_px4_bridge：px4.x = opti.x, px4.y = -opti.y, px4.z = -opti.z
    本節點做反轉換：map.x = n, map.y = -e, map.z = -d, yaw_map = -yaw_ned
- 時間戳用 ROS 系統時間（全部節點都用 use_sim_time:=false）
"""
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class Px4OdomTf(Node):
    def __init__(self):
        super().__init__('px4_odom_tf')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.declare_parameter('topic_odometry', '/fmu/out/vehicle_odometry')
        topic = self.get_parameter('topic_odometry').value
        self.create_subscription(VehicleOdometry, topic, self.cb, qos)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

    def cb(self, m: VehicleOdometry):
        n, e, d = m.position

        # orientation 暫時固定成 identity（yaw=0）：mocap rigid body 的本地軸定義
        # 目前有問題（實測水平轉機頭，Motive 顯示變化的是 roll 不是 yaw），q 算出來
        # 的 yaw 不可信。Nav2 的 costmap/MPPI critics 是直接讀這個節點發的 TF/odom
        # orientation 判斷「目前朝向」，只在 cmd_vel_bridge.py 那邊跳過旋轉不夠—
        # 這裡也要固定成identity，兩邊假設才會一致（機體座標=世界座標）。等 Motive
        # 那邊的 rigid body 軸定義校正後，要把這裡跟 cmd_vel_bridge.py 一起改回來。
        cz, sz = 1.0, 0.0

        # 機體座標=世界座標（跟上面 orientation 固定 identity 的假設一致），
        # 不需要再用 yaw 旋轉一次。
        v0, v1 = m.velocity[0], m.velocity[1]
        bx, by = v0, -v1
        wz = -m.angular_velocity[2]

        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(n)
        t.transform.translation.y = float(-e)
        t.transform.translation.z = float(-d)
        t.transform.rotation.z = sz
        t.transform.rotation.w = cz
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = float(n)
        o.pose.pose.position.y = float(-e)
        o.pose.pose.position.z = float(-d)
        o.pose.pose.orientation.z = sz
        o.pose.pose.orientation.w = cz
        o.twist.twist.linear.x = float(bx)
        o.twist.twist.linear.y = float(by)
        o.twist.twist.angular.z = float(wz)
        self.pub.publish(o)


def main():
    rclpy.init()
    node = Px4OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
