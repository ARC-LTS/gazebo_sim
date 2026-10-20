# clearpath_simulator

## Setup

Prerequisites:
  - Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Ignition Fortress

```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

### Workspace

```
mkdir ~/clearpath_ws/src -p
cd ~/clearpath_ws/src
git clone https://github.com/clearpathrobotics/clearpath_simulator.git
cd ~/clearpath_ws
rosdep install -r --from-paths src -i -y
colcon build --symlink-install
```

### Setup path

`setup_path` is the Clearpath setup directory holding `robot.yaml` and the generated
platform/sensor/manipulator launch files and URDF. It resolves, in order: an explicit
`setup_path:=` launch argument, the `RAMONES_SETUP_PATH` environment variable, then
`~/clearpath/`.

**RaMonES (this fork):** `simulation.launch.py` defaults to `generate:=false` and uses the
generated files already in `setup_path`. The dissertation campaign ran on a frozen
config generated on 2026-02-03 (notably `enable_ekf: false`, so RTAB-Map ICP odometry is
the only `odom -> base_link` publisher, and the VLP16 simulated at 10 Hz). The
`ramones_ws` meta-repo ships that config; point `RAMONES_SETUP_PATH` at it rather than
regenerating. Regenerating with a different `clearpath_generator_common` release changes
both settings and visibly corrupts the SLAM map when the robot spins.

If the generated files are missing with `generate:=false`, the launch stops with an
error naming them. With `generate:=true`, the generators overwrite the files in
`setup_path`, and the launch stops if any generator exits non-zero.

For a plain upstream setup:

```
mkdir ~/clearpath
```

Copy your `robot.yaml` into `~/clearpath` and launch once with `generate:=true`.

## Launch

```
RAMONES_SETUP_PATH=/path/to/ramones_ws/config/clearpath/ \
  ros2 launch clearpath_gz simulation.launch.py
```