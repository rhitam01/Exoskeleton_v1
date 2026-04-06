from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_root_dir = LaunchConfiguration('log_root_dir')
    run_name_pattern = LaunchConfiguration('run_name_pattern')
    csv_filename = LaunchConfiguration('csv_filename')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_root_dir',
            default_value='/home/rhitam/exoskeleton-v1/logs/hip',
            description='Root directory for hip logger output folders',
        ),
        DeclareLaunchArgument(
            'run_name_pattern',
            default_value='hip_run_{date}_{time}',
            description='Per-run folder naming pattern for logger',
        ),
        DeclareLaunchArgument(
            'csv_filename',
            default_value='hip_log.csv',
            description='CSV file name inside each run folder',
        ),

        Node(
            package='cubemars_v1',
            executable='hip_cont_pub',
            name='hip_continous',
            output='screen'
        ),

        TimerAction(
            period=0.8,
            actions=[
                Node(
                    package='cubemars_v1',
                    executable='encoder_pub',
                    name='encoder_node',
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='cubemars_v1',
                    executable='hip_logger_sub',
                    name='hip_logger',
                    output='screen',
                    parameters=[{
                        'log_root_dir': log_root_dir,
                        'run_name_pattern': run_name_pattern,
                        'csv_filename': csv_filename,
                    }]
                ),
            ]
        ),

    ])
