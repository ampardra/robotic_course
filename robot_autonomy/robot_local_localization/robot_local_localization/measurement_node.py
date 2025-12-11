#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class MeasurementNode(Node):
    def __init__(self):
        super().__init__("measurement_node")
        
        # Parameters
        self.declare_parameter("alpha", 0.5) # Weight for VO vs IMU
        self.declare_parameter("imu_topic", "/zed/zed_node/imu/data_raw")
        self.declare_parameter("vo_topic", "/vo/odom")
        
        self.alpha = self.get_parameter("alpha").value
        imu_topic = self.get_parameter("imu_topic").value
        vo_topic = self.get_parameter("vo_topic").value

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, 10)
        self.vo_sub = self.create_subscription(Odometry, vo_topic, self.vo_cb, 10)
        
        # Publisher
        self.pub = self.create_publisher(Odometry, "/measurement_model", 10)
        
        # Storage
        self.last_imu = None
        self.last_vo = None

    def imu_cb(self, msg):
        self.last_imu = msg
        self.compute_and_publish()

    def vo_cb(self, msg):
        self.last_vo = msg
        self.compute_and_publish()

    def compute_and_publish(self):
        # We need both messages to compute the "combined" model
        if self.last_imu is None or self.last_vo is None:
            return

        # Simple assignment logic: 
        # Fusing Visual Odometry Position with IMU Orientation 
        # (This is a basic model as per typical assignment requirements)
        
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        
        # 1. Take Position from Visual Odometry
        out.pose.pose.position = self.last_vo.pose.pose.position
        
        # 2. Take Orientation from IMU (Usually more accurate for pitch/roll, sometimes yaw)
        # OR: Take orientation from VO. 
        # Since the assignment asks for "Combined formulas", let's use:
        # Position = VO, Orientation = IMU
        out.pose.pose.orientation = self.last_imu.orientation
        
        # 3. Take Twist (Velocity) from VO
        out.twist = self.last_vo.twist

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()