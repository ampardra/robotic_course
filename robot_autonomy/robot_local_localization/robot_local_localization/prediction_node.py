#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')

        # 1. Parameters
        self.declare_parameter('wheel_separation', 0.45)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_pred')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 50.0)

        # 2. Get Parameters
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.pub_tf = self.get_parameter('publish_tf').value
        self.rate = self.get_parameter('update_rate').value

        # 3. State Variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_time = self.get_clock().now()

        # 4. Subscriber with BEST EFFORT (Accepts everything)
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, qos_policy)
        
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 5. Timer
        self.timer = self.create_timer(1.0 / self.rate, self.update)
        
        # DEBUG: Print startup info
        self.get_logger().warn(f"!!! DEBUG MODE !!! Listening on topic: '{self.cmd_topic}'")

    def cmd_callback(self, msg: Twist):
        # --- DEBUG LOG ---
      
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.001: return

        # --- Motion Model ---
        if abs(self.omega) < 1e-5:
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
        else:
            r = self.v / self.omega
            self.x += -r * math.sin(self.yaw) + r * math.sin(self.yaw + self.omega * dt)
            self.y += r * math.cos(self.yaw) - r * math.cos(self.yaw + self.omega * dt)
        
        self.yaw += self.omega * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Output
        q = self.euler_to_quaternion(0, 0, self.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega
        
        self.odom_pub.publish(odom_msg)

        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        # DEBUG LOOP: Uncomment this if you want to see the loop running 
        # self.get_logger().info(f"Looping: v={self.v}, x={self.x:.2f}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()