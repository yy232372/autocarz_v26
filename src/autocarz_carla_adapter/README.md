# autocarz CARLA real-topic adapter

This package lets the normal autocarz launch files run against CARLA without
adding CARLA-only code to the `autocarz` package.

## Topic bridge

The adapter converts CARLA bridge topics into the topic names used by the real
vehicle code:

- `/carla/ego_vehicle/odom` -> `/fix`
- `/carla/ego_vehicle/odom` -> `/imu/data`
- `/carla/ego_vehicle/odom` -> `/erp42/state`
- `/ctrlCmd` -> `/carla/ego_vehicle/ackermann_control_cmd`
- `/carla/ego_vehicle/lidar_vlp16/point_cloud` -> `/velodyne_points`
- `/carla/ego_vehicle/lidar_livox/point_cloud` -> `/livox/lidar`

The fake `/fix` latitude/longitude is generated so that
`sensor_data_isro.py::latlong2xy()` returns the same path-frame coordinates
published by the CARLA bridge odometry.

## Run order

Use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` in every terminal.

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Start the CARLA bridge first:

```bash
cd ~/carla_dist/CARLA_0.9.16_erp42/PythonAPI/examples/ros2
source /opt/ros/humble/setup.bash
python3 erp42_python_bridge.py --file erp42_stack.json --v
```

For CARLA, run the original autocarz launch files directly. The lidar launch
reads CARLA point cloud topics, and the vector launch starts only the small
CARLA adapter needed for `/fix`, `/imu/data`, `/erp42/state`, and Ackermann
control conversion.

```bash
cd ~/autocarz_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autocarz autocarz_lidar.launch.py carla:=true
```

```bash
cd ~/autocarz_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autocarz autocarz_vector.launch.py carla:=true
```

`Stereo is NOT SUPPORTED` from RViz is informational and does not stop driving.
Do not start `autocarz_vector.launch.py` twice.

If you specifically want to test the real serial node in CARLA, create a fake
ERP42 serial device before starting the vector launch:

```bash
sudo -E env PYTHONPATH=$PYTHONPATH PATH=$PATH \
  ros2 run autocarz_carla_adapter fake_erp42_serial
```

Then run the normal launch files in separate terminals:

```bash
ros2 launch autocarz autocarz_lidar.launch.py carla:=true
ros2 launch autocarz autocarz_vector.launch.py carla:=true use_serial:=true
```
