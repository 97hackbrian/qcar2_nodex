# qcar2_nodex
Qcar2 start node modified usign launch files.

Just run the next command before compile the new qcar2_nodex package and clone this repo.
```bash
ros2 launch qcar2_nodex qcar2_virtual_launch.py camera_ids_config:="0, 1, 2, 3"
```
The expected output is:

```bash
admin@raider:/workspaces/isaac_ros-dev/ros2$ ros2 topic list
/camera/color_image
/camera/color_image/compressed
/camera/color_image/compressedDepth
/camera/color_image/theora
/camera/csi_image_0
/camera/csi_image_0/compressed
/camera/csi_image_0/compressedDepth
/camera/csi_image_0/theora
/camera/csi_image_1
/camera/csi_image_1/compressed
/camera/csi_image_1/compressedDepth
/camera/csi_image_1/theora
/camera/csi_image_2
/camera/csi_image_2/compressed
/camera/csi_image_2/compressedDepth
/camera/csi_image_2/theora
/camera/csi_image_3
/camera/csi_image_3/compressed
/camera/csi_image_3/compressedDepth
/camera/csi_image_3/theora
/camera/depth_image
/camera/depth_image/compressed
/camera/depth_image/compressedDepth
/camera/depth_image/theora
/parameter_events
/qcar2_battery
/qcar2_imu
/qcar2_joint
/qcar2_led_cmd
/qcar2_motor_speed_cmd
/rosout
/scan

```
