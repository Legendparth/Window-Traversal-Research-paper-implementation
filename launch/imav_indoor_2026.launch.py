import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

TERMINAL = 'gnome-terminal'

PX4_AUTOPILOT_DIR = os.path.expanduser('~/PX4-Autopilot')
QGC_DIR = os.path.expanduser('~/Downloads')

def generate_launch_description():

    px4_command = (
        f'cd {PX4_AUTOPILOT_DIR} && '
        'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Tools/simulation/gz/models && '
        'PX4_GZ_WORLD=imav2026_scaled PX4_GZ_MODEL_POSE="0,-6.5,0" make px4_sitl gz_x500_depth'
    )
    px4_sitl = ExecuteProcess(
        cmd=[TERMINAL, '--title="PX4 SITL"', '--', 'bash', '-c', px4_command],
        output='log',
        name='px4_sitl'
    )

    microxcre_agent = ExecuteProcess(
        cmd=[TERMINAL, '--title="MicroXRCE Agent"', '--', 'MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='log',
        name='microxcre_agent'
    )

    qgc_command = f'cd {QGC_DIR} && ./QGroundControl-x86_64.AppImage'
    qgc_launch = ExecuteProcess(
        cmd=[TERMINAL, '--title="QGroundControl"', '--', 'bash', '-c', qgc_command],
        output='log',
        name='qgroundcontrol'
    )

    gz_bridge_cmd = (
        'ros2 run ros_gz_bridge parameter_bridge '
        '/world/imav2026_scaled/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image '
    )
    ros_gz_bridge = ExecuteProcess(
        cmd=[TERMINAL, '--title="ROS GZ Bridge"', '--', 'bash', '-c', gz_bridge_cmd],
        output='log',
        name='ros_gz_bridge'
    )

    drone_control_cmd = 'ros2 run imav_indoor_2026 drone_joystick_cam'
    drone_control = ExecuteProcess(
        cmd=[TERMINAL, '--title="Drone Controller"', '--', 'bash', '-c', drone_control_cmd],
        output='log',
        name='drone_node'
    )
    

    # --- 8. Return Launch Description ---
    return LaunchDescription([
        px4_sitl,
        microxcre_agent,
        qgc_launch,
        ros_gz_bridge,
        drone_control,
    ])