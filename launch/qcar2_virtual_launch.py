# This is the NEW MODIFIED launch file that starts up the basic QCar2 nodes FOR ALL CAMERAS

import subprocess

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, TimerAction)
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import (OnProcessExit, OnProcessStart)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    
    
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
    

    csi_camera_nodes = []
    for i in range(4):
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
    
    qcar2_hardware = Node(
            package='qcar2_nodes',
            executable='qcar2_hardware',
            name='qcar2_hardware',
            parameters=[{"device_type":"virtual"}]

        )
     
    return LaunchDescription([
        lidar_node,
        realsense_camera_node,
        *csi_camera_nodes,
        qcar2_hardware,
    ])
