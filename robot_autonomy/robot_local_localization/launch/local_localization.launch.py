import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Simulation ---
    robot_desc_pkg = get_package_share_directory('robot_description')
    gazebo_launch_path = os.path.join(robot_desc_pkg, 'launch', 'gazebo.launch.py')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    # --- 2. Prediction Node ---
    prediction_node = Node(
        package='robot_local_localization',
        executable='prediction_node',
        name='prediction_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_tf': True,      # Publishes odom -> base_link
            'cmd_vel_topic': '/cmd_vel',
            'wheel_separation': 0.45,
            'wheel_radius': 0.1
        }]
    )

    # --- 3. Measurement Node ---
    measurement_node = Node(
        package='robot_local_localization',
        executable='measurement_node',
        name='measurement_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'imu_topic': '/zed/zed_node/imu/data_raw',
            'vo_topic': '/vo/odom'
        }]
    )

    # --- 4. EKF Node ---
    ekf_node = Node(
        package='robot_local_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # Use TimerAction to delay your nodes slightly to let Gazebo start first
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[prediction_node, measurement_node]
    )

    return LaunchDescription([
        simulation,
        delayed_nodes
    ])