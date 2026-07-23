from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    enable_deadman_arg = DeclareLaunchArgument(
        'enable_deadman',
        default_value='true',
        description='Whether the deadman button gate is enforced by safety_node'
    )

    use_fallback_follow_method = DeclareLaunchArgument(
        'use_fallback_follow_method',
        default_value='false',
        description='Whether or not the gap follow node should use the default fallback method'
    )
    
    return LaunchDescription([
        enable_deadman_arg,
        use_fallback_follow_method,

        # Launch gap_finder_node
        Node(
            package='reactive',
            executable='gap_finder_node',
            name='gap_finder_node',
            output='screen',
            remappings=[('drive', 'drive_raw')],
        ),

        # Launch gap_follower_node
        Node(
            package='reactive',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            remappings=[('drive', 'drive_raw')],
            parameters=[{'use_fallback_method': LaunchConfiguration('use_fallback_follow_method')}]
        ),

        # Launch safety_node
        Node(
            package='reactive',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[{'enable_deadman': LaunchConfiguration('enable_deadman')}],
        )
    ])
