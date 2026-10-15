# TESSE Foxglove visualization

This ROS 2 package visualizes TESSE's quadrotor state and the controller's requested trajectory in Foxglove. It subscribes to `/current_state` (`nav_msgs/Odometry`) and `/desired_state` (`trajectory_msgs/MultiDOFJointTrajectoryPoint`) and publishes body, target, and bounded trail markers on `/tesse/visuals`.

The existing `tesse_ros_bridge` launch remaps `odom` to `/current_state`, publishes `world -> base_link_gt`, and publishes `/clock`. The trajectory publisher supplies `/desired_state` and `world -> av-desired`. This package does not start TESSE or control the vehicle.

## Run

```bash
colcon build --packages-select tesse_foxglove_pkg
source install/setup.bash
ros2 launch tesse_foxglove_pkg tesse_foxglove.launch.py
```

Start TESSE and its ROS bridge separately. Connect Foxglove to `ws://localhost:8765`, then import `config/tesse_foxglove.json` as a layout. The 3D panel follows TESSE's `base_link_gt` frame. The Plot panel compares X/Y/Z positions from `/current_state` and `/desired_state`, and the Image panel reads `/third_person/rgb/image_raw`.

In `tesse_ros_bridge/config/params.yaml`, set `publish_third_pov: true` before starting the TESSE bridge to populate the Image panel. The repository currently defaults it to `false`.

Launch arguments `odom_topic`, `desired_topic`, `visuals_topic`, and `use_sim_time` can be overridden to match a different ROS graph. If you change `visuals_topic` or `odom_topic`, update the layout's topic or plot paths accordingly.
