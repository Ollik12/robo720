# Exercise 5: Visual servoing

To get the latest changes in the repo to your local directory, navigate to the course directory in your terminal, run `git pull`, and resolve all possible merge conflicts (you may save your own edits, but in the future it is possible that the templates will not work with your code). If you want to save your own code separately and just get the fresh template, navigate to your home directory and run:

```bash
git clone https://github.com/tau-alma/robo720_2026.git
```

Once inside container again, navigate to `ros2_ws` and build and source the packages:

```bash
cd ~/ros2_ws
colcon build --parallel-workers $(($(nproc)/2))
source install/setup.bash
```

## Launch setup

To launch the visual servoing controller and pose estimator, run:

```bash
ros2 launch ex5 robot.launch.py
```
By default, a received `goal_pose` is followed; if no goal has been received, the controller follows the wave. Enable marker tracking with:

```bash
ros2 param set --no-daemon /visual_controller use_marker_top_pose true
```
In marker mode, the controller follows the wave until the marker is detected. Disable marker mode to return to the goal pose or wave:
```bash
ros2 param set --no-daemon /visual_controller use_marker_top_pose false
```

The gripper can be operated manually with its action controller. Close and open it with:

```bash
ros2 action send_goal /hand_controller/gripper_cmd control_msgs/action/GripperCommand "{position: 0.0, max_effort: 20.0}"
ros2 action send_goal /hand_controller/gripper_cmd control_msgs/action/GripperCommand "{position: 0.04, max_effort: 20.0}"
```

Automatic pick-and-place is not implemented yet; it needs a configured placement pose.

So got to pose o, enable use marker pose then we go on top of the marker