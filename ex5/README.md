# Final demo
So in this final demo I have implemented marker tracking based on exercise 5 functionality. So we get cam_to_marker frame from a box with an aruco marker on top with a camera installed in the robot's wrist. Then we transform it to base_to_marker so we can give marker's position as a goal pose to our robot and we add a marker approach height of 7cm.

General robot motion control is implemented as attractive torque
with cartesian task-space controller with joint space torque control so what is used in exercise 5.

We have also implemented repulsive torque control based on exercise 6. So we have joint limits as artificial potential field and the actual obstacles as objects to avoid. These all are included in tau_rep and added to total torque command. In addition to this, we calculate Null space stabilizing torque as tau_null and add that also to the total torque command. This makes sure that some of the joints dont move when EE is in the goal pose.

So with using these functionalities the idea of this final demo is to have a workcell around the robot, use the camera to find the boxes, go on top of boxes and then grab the boxes for pick and place task. But in the workcell, someone has placed two beams that are in the way of our box placing goals so this is why we have obstacle avoidance. Gripper is included as the end effector.


```bash
cd ~/ros2_ws
colcon build --parallel-workers $(($(nproc)/2))
source install/setup.bash
```

## Launch setup

To launch the visual servoing controller and pose estimator, run:

```bash
source install/setup.bash
ros2 launch ex5 robot.launch.py
```

```bash
source install/setup.bash
ros2 run goal_publishers goal_pose_publisher.py
```
By default, a received `goal_pose` is followed; if no goal has been received, the controller follows the wave. Enable marker tracking with:

initially set goalpose to o

Then open or close the gripper:

```bash
source install/setup.bash
ros2 run ex5 gripper_command.py close
ros2 run ex5 gripper_command.py open
```

```bash
source install/setup.bash
ros2 param set --no-daemon /visual_controller use_marker_top_pose true
ros2 param set /visual_controller use_marker_top_pose true
```
In marker mode, the controller follows the wave until the marker is detected. Disable marker mode to return to the goal pose or wave:
```bash
ros2 param set --no-daemon /visual_controller use_marker_top_pose false
```


So got to pose o, enable use marker pose then we go on top of the marker automatically.
