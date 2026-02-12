# This is the NEW MODIFIED launch file that starts up the basic QCar2 nodes FOR ALL CAMERAS

import subprocess

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, TimerAction, DeclareLaunchArgument)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import (OnProcessExit, OnProcessStart)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    camera_ids_config = LaunchConfiguration('camera_ids_config').perform(context)
    
    try:
        # Simple parsing to handle "3" or "3, 1" or "[3]"
        cleaned = camera_ids_config.strip('[]"\' ')
        if not cleaned:
             numbers = []
        else:
             numbers = [int(x.strip()) for x in cleaned.split(',')]
    except ValueError:
        # Fallback if parsing fails
        print(f"Warning: Could not parse camera_ids_config='{camera_ids_config}'. Defaulting to [3].")
        numbers = [3]

    csi_camera_nodes = []

    for i in numbers:
        node = Node(
            package='qcar2_nodes',
            executable='csi',
            # Es importante que cada nodo tenga un nombre único
            name=f'csi_camera_{i}',
            remappings=[
                ('/camera/csi_image', f'/camera/csi_image_{i}'),
                ('/camera/csi_image/compressed', f'/camera/csi_image_{i}/compressed'),
                ('/camera/csi_image/compressedDepth', f'/camera/csi_image_{i}/compressedDepth'),
                ('/camera/csi_image/theora', f'/camera/csi_image_{i}/theora')
            ],
            parameters=[{
                "device_type": "virtual",
                "frame_width": 410,
                "frame_height": 205,
                "frame_rate": 15.0,
                "camera_num": i  
            }]
        )
        csi_camera_nodes.append(node)
    
    return csi_camera_nodes


def generate_launch_description():

    camera_ids_config_arg = DeclareLaunchArgument(
        'camera_ids_config',
        default_value='3',
        description='List of camera IDs to launch (e.g., "3" or "0, 1, 2, 3")'
    )
    
    lidar_node = Node(
            package='qcar2_nodes',
            executable='lidar',
            name='Lidar',
            parameters=[{"device_type":"virtual"}]
        )
    
    realsense_camera_node = Node(
            package='qcar2_nodes',
            executable='rgbd',
            name='RealsenseCamera',
            parameters=[{"device_type":"virtual"},
                        {"frame_width_rgb":640},
                        {"frame_height_rgb":480},
                        {"frame_width_depth":640},
                        {"frame_height_depth":480}]
        )
    
    qcar2_hardware = Node(
            package='qcar2_nodes',
            executable='qcar2_hardware',
            name='qcar2_hardware',
            parameters=[{"device_type":"virtual"}]

        )
     
    return LaunchDescription([
        camera_ids_config_arg,
        lidar_node,
        realsense_camera_node,
        qcar2_hardware,
        OpaqueFunction(function=launch_setup)
    ])
