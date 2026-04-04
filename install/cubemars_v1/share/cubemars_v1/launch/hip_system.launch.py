from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='cubemars_v1',
            executable='hip_cont_pub',
            name='hip_continous',
            output='screen'
        ),

        Node(
            package='cubemars_v1',
            executable='encoder_pub',
            name='encoder_node',
            output='screen'
        ),

        Node(
            package='cubemars_v1',
            executable='hip_logger_sub',
            name='hip_logger',
            output='screen'
        ),

    ])
