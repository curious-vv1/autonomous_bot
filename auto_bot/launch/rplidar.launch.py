import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    rplidar = Node(package="rplidar_ros",executable="rplidar_composition",output="screen",parameters=[{
        "serial_port":"/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0",
        "frame_id":"laser_link",
        "angle_compensate":"True",
        "scan_mode":"Standard"
    }])

    # This is the launch code to be used using raspberry pi connected to lidar with path mentioned in serial port

    return LaunchDescription([
        rplidar
    ])
