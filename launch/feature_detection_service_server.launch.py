from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_tf_project',
            executable='feature_detection_service_server',
            name='feature_detection_service_server',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', 'install/ros2_tf_project/share/ros2_tf_project/rviz/tb3_burger.rviz'],
            output='screen'
        )
    ])