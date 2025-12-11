#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import math

class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')

        # 1. Declare Parameters
        self.declare_parameter('wheel_separation', 0.45)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_pred')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 50.0)

        # 2. Get Parameters
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.pub_tf = self.get_parameter('publish_tf').value
        self.rate = self.get_parameter('update_rate').value

        # 3. State Variables [x, y, yaw]
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # 4. Input Variables
        self.v = 0.0      # Linear velocity
        self.omega = 0.0  # Angular velocity
        self.last_time = self.get_clock().now()

        # 5. Subscribers & Publishers
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 6. Timer
        self.timer = self.create_timer(1.0 / self.rate, self.update)
        
        self.get_logger().info("Prediction Node Started (No tf_transformations dep)")

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0: return

        # --- Motion Model (Unicycle) ---
        # If omega is very small, use straight line approximation
        if abs(self.omega) < 1e-5:
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
        else:
            # Exact integration
            r = self.v / self.omega
            self.x += -r * math.sin(self.yaw) + r * math.sin(self.yaw + self.omega * dt)
            self.y += r * math.cos(self.yaw) - r * math.cos(self.yaw + self.omega * dt)
        
        self.yaw += self.omega * dt
        
        # Normalize Yaw (-pi to pi)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # --- Convert Yaw to Quaternion (Manual Math) ---
        q = self.euler_to_quaternion(0, 0, self.yaw)

        # --- Publish Odometry ---
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

        # --- Publish TF ---
        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Manually convert Euler angles to Quaternion to avoid dependency issues.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PredictionNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in Prediction Node: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()