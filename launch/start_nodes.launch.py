from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Prefix to run nodes under sudo while preserving environment variables
    sudo_prefix = [
        "sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "
    ]

    return LaunchDescription([
        Node(
            package='wu25',
            executable='bno_node',
            name='bno_node',
            output='screen',
            prefix=sudo_prefix,
            shell=True,
        ),
        Node(
            package='wu25',
            executable='thrusters',
            name='thrusters',
            output='screen',
            prefix=sudo_prefix,
            shell=True,
        ),
        # Start the ROS->HTTP bridge script as part of the launch (with sudo -E env)
        ExecuteProcess(
            cmd=[
                'sudo', '-E', 'env', 'PYTHONPATH=$PYTHONPATH', 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH',
                'PATH=$PATH', 'USER=$USER', 'bash', '-lc',
                "source ~/ros2_ws/install/local_setup.bash >/dev/null 2>&1; export ROS_DOMAIN_ID=42; nohup python3 /home/pi/ros2_ws/src/WU25/tools/ros_to_http.py >/tmp/ros_to_http.log 2>&1 &"
            ],
            output='screen',
        ),
    ])
