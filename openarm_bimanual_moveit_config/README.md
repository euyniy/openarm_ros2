# MoveIt2 on Bimanual Openarms

Ensure the ROS2 packages and dependencies are installed by following the instructions in `openarm_ros2/README.md`.

## Physical Hardware

1. Initialize CAN devices for bimanual setup:
   ```sh
   # Run the bimanual CAN initialization script
   ./openarm_bimanual_bringup/utils/init_bimanual_can.sh /dev/ttyACM0 /dev/ttyACM1 can0 can1
   ```
   By default, can0 is the right arm and can1 is the left arm, but this can be adjusted in the ros2_control definition.

2. Launch the bimanual hardware:
   ```sh
   ros2 launch openarm_bimanual_bringup openarm_bimanual.launch.py
   ```

3. Optionally, start the head-mounted realsense camera. This enables the octomap occupancy grid for planning around obstacles.
   ```sh
   ros2 launch openarm_bimanual_bringup depth_camera.launch.py
   ```

## Launch the demo

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

## Alternative: Simulation Mode

For testing without hardware:
```sh
ros2 launch openarm_bimanual_bringup openarm_bimanual.launch.py use_mock_hardware:=true
```
