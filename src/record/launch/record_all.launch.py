from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.realpath(__file__))
    base_dir = os.path.dirname(launch_dir)
    output_dir = os.path.join(base_dir, 'bag_output')

    os.makedirs(output_dir, exist_ok=True)

    name_case = 'navis_CASE_ProceedCaution'

    bag_sensor = os.path.join(output_dir, name_case)
    bag_video  = os.path.join(output_dir, f'{name_case}_vid')

    # ===== ROSBAG SENSOR (CUSTOM + NON CUSTOM) =====
    rosbag_sensor = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/reeman/pose',
            '/uwb/person_pos',
            '/uwb/pose',
            '/button/case_state',
            '/button/mode',
            '/dual_leg',
            '--output', bag_sensor
        ],
        output='screen'
    )

    # ===== ROSBAG VIDEO =====
    rosbag_video = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/vision/hand_action_ready',
            '/vision/hand_frame',
            '/cmd/hand_stop',
            '--output', bag_video
        ],
        output='screen'
    )

    # ===== CSV LOGGER (C++ NODE, CUSTOM MSG ONLY) =====
    csv_node = Node(
        package='record',
        executable='csv_logger',
        output='screen'
    )

    return LaunchDescription([
        rosbag_sensor,
        rosbag_video,
        csv_node
    ])

