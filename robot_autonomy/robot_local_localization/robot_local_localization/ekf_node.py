#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import numpy as np

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # --- Parameters ---
        # Covariance for Process Noise (Q) - How much we trust the Motion Model
        # Low = Trust Physics high, High = Physics is noisy
        self.q_var_x = 0.01
        self.q_var_y = 0.01
        self.q_var_theta = 0.01

        # Covariance for Measurement Noise (R) - How much we trust the Sensors
        self.r_var_x = 0.1
        self.r_var_y = 0.1
        self.r_var_theta = 0.1

        # --- State Definitions ---
        # State Vector X = [x, y, theta]
        self.X = np.zeros((3, 1)) 
        
        # Covariance Matrix P (Initial uncertainty)
        self.P = np.eye(3) * 0.1

        # Process Noise Matrix Q
        self.Q = np.diag([self.q_var_x, self.q_var_y, self.q_var_theta])

        # Measurement Noise Matrix R
        self.R = np.diag([self.r_var_x, self.r_var_y, self.r_var_theta])

        # Measurement Matrix H (We measure [x, y, theta] directly, so Identity)
        self.H = np.eye(3)

        # --- Timers & Variables ---
        self.last_time = self.get_clock().now()
        self.first_run = True

        # --- QoS ---
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # --- Subscribers ---
        # 1. Motion Model (Prediction) - Acts as the "Control" step
        self.create_subscription(Odometry, '/odom_pred', self.prediction_callback, qos_policy)
        
        # 2. Measurement Model (Correction) - Acts as the "Update" step
        self.create_subscription(Odometry, '/measurement_model', self.measurement_callback, qos_policy)

        # --- Publisher ---
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        self.get_logger().info("EKF Node Started")

    def prediction_callback(self, msg: Odometry):
        """
        Prediction Step (Time Update):
        X_pred = Motion Model Output
        P_pred = F * P * F.T + Q
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if self.first_run:
            self.first_run = False
            return
        
        # 1. Get Control Inputs from the message
        v = msg.twist.twist.linear.x
        # We extract theta from the quaternion in the message
        _, _, yaw_pred = self.quaternion_to_euler(msg.pose.pose.orientation)

        # 2. State Prediction
        # Since prediction_node already integrated the physics, we can trust its X calculation 
        # as our "A Priori" estimate.
        # However, to be a true EKF, we usually propagate P based on the Jacobian F.
        
        theta = self.X[2, 0] # Previous theta
        
        # Update State Vector X based on the incoming prediction message
        self.X[0, 0] = msg.pose.pose.position.x
        self.X[1, 0] = msg.pose.pose.position.y
        self.X[2, 0] = yaw_pred

        # 3. Jacobian F (Linearized Motion Model)
        # F = [[1, 0, -v*sin(th)*dt],
        #      [0, 1,  v*cos(th)*dt],
        #      [0, 0,  1           ]]
        
        F = np.eye(3)
        F[0, 2] = -v * math.sin(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        
        # 4. Predict Covariance
        # P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

        # Publish the "Predicted" state (Optional, but good for visualization smoothness)
        # We usually publish only after correction, but if measurements are slow, 
        # we can publish here too. For this assignment, let's wait for measurement or publish both.
        # self.publish_ekf()

    def measurement_callback(self, msg: Odometry):
        """
        Correction Step (Measurement Update):
        K = P * H.T * inv(H * P * H.T + R)
        X = X + K * (Z - H * X)
        P = (I - K * H) * P
        """
        # 1. Extract Measurement Vector Z
        _, _, yaw_meas = self.quaternion_to_euler(msg.pose.pose.orientation)
        Z = np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y],
                      [yaw_meas]])

        # 2. Calculate Innovation (Error between Meas Z and Pred X)
        # Y = Z - H * X
        Y = Z - (self.H @ self.X)

        # IMPORTANT: Normalize angle difference to -pi to pi
        Y[2, 0] = math.atan2(math.sin(Y[2, 0]), math.cos(Y[2, 0]))

        # 3. Calculate Innovation Covariance S
        # S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # 4. Calculate Kalman Gain K
        # K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 5. Update State X
        # X = X + K * Y
        self.X = self.X + (K @ Y)

        # Normalize Theta in X again
        self.X[2, 0] = math.atan2(math.sin(self.X[2, 0]), math.cos(self.X[2, 0]))

        # 6. Update Covariance P
        # P = (I - K * H) * P
        I = np.eye(3)
        self.P = (I - K @ self.H) @ self.P

        # 7. Publish Result
        self.publish_ekf()

    def publish_ekf(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link_ekf" # Distinct frame to visualize in RViz

        msg.pose.pose.position.x = float(self.X[0, 0])
        msg.pose.pose.position.y = float(self.X[1, 0])
        
        q = self.euler_to_quaternion(0, 0, float(self.X[2, 0]))
        msg.pose.pose.orientation = q

        # Optional: Fill covariance in msg
        # msg.pose.covariance[0] = self.P[0,0] ...

        self.ekf_pub.publish(msg)

    # --- Helper Math Functions ---
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def quaternion_to_euler(self, q):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()