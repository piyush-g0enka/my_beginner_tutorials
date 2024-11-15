import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import time

def generate_launch_description():
    return LaunchDescription([
        # Declare the argument to enable or disable bag recording
        DeclareLaunchArgument(
            'record_bag', 
            default_value='true', 
            description='Whether to record all topics to a bag file (true/false)'
        ),

        # Set up the environment variable to control recording
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],  # -a means record all topics
            condition=launch.conditions.LaunchConfigurationEquals('record_bag', 'true'),
            name='rosbag_record',
            output='screen'
        ),

        # Add an action to print a message when recording starts
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('record_bag', 'true'),
            msg="Recording all topics to a bag file for 15 seconds..."
        ),

        # Set a timer to stop the bag recording after 15 seconds
        ExecuteProcess(
            cmd=['sleep', '15'],  # Sleep for 15 seconds
            name='sleep_for_15_seconds',
            output='screen'
        ),

        # Stop the bag recording after 15 seconds
        ExecuteProcess(
            cmd=['ros2', 'bag', 'stop'],
            condition=launch.conditions.LaunchConfigurationEquals('record_bag', 'true'),
            name='rosbag_stop',
            output='screen',
            emulate_tty=True
        ),

        # Add an action to log completion
        LogInfo(
            msg="Bag recording completed, stopping the bag file."
        ),
    ])
