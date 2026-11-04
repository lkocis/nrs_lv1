# NRS - LV1

ROS 2 Jazzy workspace for the UR5 cabinet-opening exercise.

- `ur5_workcell_*`: robot description, hardware bringup, Gazebo simulation and MoveIt configuration.
- `robotiq_3f_*`: articulated gripper model, drivers and controllers.
- `robotiq_ft*`: FT300S model and driver.
- `cabinet_generator`: cabinet model and randomized placement on the table.
- `nrs_lv1_bringup`: simulation and hardware launch files.

Build and run the docker container:
```bash
./docker/build.sh
./docker/run.sh
```

The workspace is mounted under `/workspaces/nrs_lv1`.; 

Build and source the workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

Simulation launch:
```bash
ros2 launch nrs_lv1_bringup simulation.launch.py
```

Hardware launch:
```bash
ros2 launch nrs_lv1_bringup hardware.launch.py
```
