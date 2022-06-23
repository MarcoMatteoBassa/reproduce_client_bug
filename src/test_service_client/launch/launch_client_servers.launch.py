from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_service_client',
            executable='service_server',
            name='service_server_1',
            parameters=[{'service_name': 'service_server_1'}]
        ),
        Node(
            package='test_service_client',
            executable='service_server',
            name='service_server_2',
            parameters=[{'service_name': 'service_server_2'}]
        ),
        Node(
            package='test_service_client',
            executable='service_server',
            name='service_server_3',
            parameters=[{'service_name': 'service_server_3'}]
        ),
        Node(
            package='test_service_client',
            executable='service_server',
            name='service_server_4',
            parameters=[{'service_name': 'service_server_4'}]
        ),
        Node(
            package='test_service_client',
            executable='service_client',
            name='service_client',
            parameters=[{'servers_names': ['service_server_1', 'service_server_2', 'service_server_3', 'service_server_4']}]
        )
    ])
