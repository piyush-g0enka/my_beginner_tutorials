from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        # Declare the 'publish_frequency' argument (with a default value of 10 Hz)
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='10.0',  # Default value is 10 Hz
            description='Frequency at which the talker node publishes messages to the /chatter topic'
        ),

        # Launch the 'talker' node and pass the publish_frequency argument as a parameter
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker_node',
            output='screen',
            parameters=[{'publish_frequency': LaunchConfiguration('publish_frequency')}]
        ),
        
        # Launch the 'listener' node (no parameter needed)
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener_node',
            output='screen'
        )
    ])
