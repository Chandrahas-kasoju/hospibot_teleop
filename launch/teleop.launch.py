from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Rosbridge
    # We assume rosbridge_server is installed. 
    # If not, the user needs to install it: sudo apt install ros-<distro>-rosbridge-server
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        ])
    )
    
    # Web Server Node
    web_server_node = Node(
        package='hospibot_teleop',
        executable='web_server',
        name='web_server',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        web_server_node
    ])
