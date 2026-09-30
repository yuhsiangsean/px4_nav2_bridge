#!/usr/bin/env python3
"""
PX4 vehicle_odometry -> Nav2 需要的 TF(odom -> base_link) 與 /odom.

- 只取平面資訊（x, y, yaw），高度照抄，roll/pitch 忽略（Nav2 是 2D）
- 座標慣例與 mocap_px4_bridge 一致，讓 map = OptiTrack 世界座標（X 前、Y 左、Z 上）：
    mocap_px4_bridge：px4.x = opti.x, px4.y = -opti.y, px4.z = -opti.z
    本節點做反轉換：map.x = n, map.y = -e, map.z = -d, yaw_map = -yaw_ned
- 時間戳用 ROS 系統時間（全部節點都用 use_sim_time:=false）
"""
import math

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
        w, qx, qy, qz = m.q
        yaw_ned = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        yaw = -yaw_ned
        cz, sz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        c, s = math.cos(yaw), math.sin(yaw)

        # 速度轉到機體 FLU
        v0, v1 = m.velocity[0], m.velocity[1]
        if m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            bx, by = v0, -v1
        else:  # NED 世界座標 -> map -> 機體
            vx, vy = v0, -v1
            bx = c * vx + s * vy
            by = -s * vx + c * vy
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
