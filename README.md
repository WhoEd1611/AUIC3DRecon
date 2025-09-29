# AUIC3DRecon
3D Reconstruction Code for AUIC

Created by: Edric Lay
Date Created: 17/09/25

## To use:
### Hardware
- Plug arduino into PC via USB
- Plug servo signal wire into Pin 9 and ensure servo is powered
- Hit reset button on arduino to zero

### Terminal one:
'''
sudo chmod a+rw /dev/ttyACM0
ros2 run arduino_serial serial_node
'''

### Terminal two:
'''
ros2 run point_cloud_blender icpBlender
'''

### Terminal three:
'''
rviz2
'''
Load the config called d435.rviz

### Terminal four:
'''
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
'''
Load the point cloud camera