#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # --- Parameters ---
        self.declare_parameter('square_side', 2.0) # meters
        self.declare_parameter('speed', 0.2)       # m/s
        self.side_length = self.get_parameter('square_side').value
        self.speed = self.get_parameter('speed').value

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Path Publishers (For Visualization)
        self.path_pub_pred = self.create_publisher(Path, '/path/prediction', 10)
        self.path_pub_meas = self.create_publisher(Path, '/path/measurement', 10)
        self.path_pub_ekf = self.create_publisher(Path, '/path/ekf', 10)
        self.path_pub_real = self.create_publisher(Path, '/path/ground_truth', 10)

        # --- Path Storage ---
        self.path_pred = Path()
        self.path_meas = Path()
        self.path_ekf = Path()
        self.path_real = Path()

        # Set Frames
        self.path_pred.header.frame_id = "odom"
        self.path_meas.header.frame_id = "odom"
        self.path_ekf.header.frame_id = "odom"
        self.path_real.header.frame_id = "odom"

        # --- Subscribers ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        self.create_subscription(Odometry, '/odom_pred', self.cb_pred, qos)
        self.create_subscription(Odometry, '/measurement_model', self.cb_meas, qos)
        self.create_subscription(Odometry, '/ekf_odom', self.cb_ekf, qos)
        # Gazebo Ground Truth (Adjust topic name if needed)
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_real, qos) 

        # --- Control Logic Variables ---
        self.state = 0 # 0: Move, 1: Turn
        self.start_time = self.get_clock().now()
        self.duration_move = self.side_length / self.speed
        self.duration_turn = (math.pi / 2.0) / 0.2 # Assuming 0.2 rad/s turn speed
        
        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Test Node Started: Drawing Rectangle")

    # --- Callbacks to store and publish paths ---
    def append_pose(self, msg, path_obj, pub):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        path_obj.poses.append(pose)
        
        # Keep path from getting infinite
        if len(path_obj.poses) > 5000:
            path_obj.poses.pop(0)
            
        pub.publish(path_obj)

    def cb_pred(self, msg): self.append_pose(msg, self.path_pred, self.path_pub_pred)
    def cb_meas(self, msg): self.append_pose(msg, self.path_meas, self.path_pub_meas)
    def cb_ekf(self, msg): self.append_pose(msg, self.path_ekf, self.path_pub_ekf)
    def cb_real(self, msg): self.append_pose(msg, self.path_real, self.path_pub_real)

    # --- Robot Control State Machine ---
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds / 1e9
        
        msg = Twist()

        if self.state == 0: # Move Straight
            if dt < self.duration_move:
                msg.linear.x = self.speed
                msg.angular.z = 0.0
            else:
                self.state = 1 # Switch to turn
                self.start_time = now
        
        elif self.state == 1: # Turn 90 deg
            if dt < self.duration_turn:
                msg.linear.x = 0.0
                msg.angular.z = 0.2 
            else:
                self.state = 0 # Switch to move
                self.start_time = now

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()