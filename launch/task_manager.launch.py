import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # IC120のクローラ走行・自己位置推定(EKF)用
    ic120_standby_launch = os.path.join(
            get_package_share_directory('ic120_unity'),
            'launch',
            'ic120_standby_ekf.launch.py'
            )

    ic120_navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource( ic120_standby_launch)
            )

    dump_controller = TimerAction(
            period = 30.0,
            actions=[
                ExecuteProcess(
                        cmd = ['ros2','run','task_manager','dump_release_controller'],
                        output = 'screen'
                    )
                ]
            )
    zx200_arm_brain = TimerAction(
            period = 35.0,
            actions = [
                Node(
                    package='zx200_autonomy',
                    executable = 'state_machine_node',
                    output = 'screen'
                    )
                ]
            )
    waypoint_manager = TimerAction(
            period = 40.0,
            actions = [
                Node(
                    package='track_manager',
                    executable = 'waypoint_server',
                    name = 'waypoint_server',
                    output = 'screen'
                    )
                ]
            )

    return LaunchDescription([
        ic120_navigation,
        dump_controller,
        zx200_arm_brain,
        waypoint_manager
        ])
