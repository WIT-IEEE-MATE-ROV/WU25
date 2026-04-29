from launch import LaunchDescription
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
    ])
