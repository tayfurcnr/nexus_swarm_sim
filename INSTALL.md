# Installation Guide

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

![Nexus Swarm Sim Logo](docs/logo.png)

This guide describes the recommended installation and validation flow for `nexus_swarm_sim` on a clean machine.

## Table Of Contents

- [Deployment Model](#deployment-model)
- [What The Setup Script Installs](#what-the-setup-script-installs)
- [Recommended Fresh Install](#recommended-fresh-install)
- [Pre-Launch Validation](#pre-launch-validation)
- [Successful Install Checklist](#successful-install-checklist)
- [Recommended Bringup Sequence](#recommended-bringup-sequence)
- [Dependency Recovery](#dependency-recovery)
- [Model Sources By Launch Mode](#model-sources-by-launch-mode)
- [Gimbal Asset Note](#gimbal-asset-note)

## Deployment Model

Target environment:
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11 Classic
- ArduPilot SITL
- MAVROS

The recommended filesystem layout is:
- `nexus_swarm_sim` inside a catkin workspace
- `ardupilot` in `~/ardupilot`
- `ardupilot_gazebo` in `~/ardupilot_gazebo`

Reference layout:

```text
~/nexus_swarm_sim_ws/
  src/
    nexus_swarm_sim/

~/ardupilot/
~/ardupilot_gazebo/
```

This layout matches the current runtime expectations used by the launch files and helper scripts.

## What The Setup Script Installs

Running [setup_ardupilot_noetic.sh](/home/tayfurcnr/swarm_ws/src/nexus_swarm_sim/setup_ardupilot_noetic.sh) prepares the following:
- ROS, Gazebo, and MAVROS-related system packages
- GeographicLib datasets required by MAVROS
- Python dependencies from `requirements.txt`
- `~/ardupilot` if missing
- `~/ardupilot_gazebo` if missing
- the catkin workspace build for `nexus_swarm_sim`

The setup script also supports:
- interactive approval for long-running dependency steps
- `--yes` for unattended or non-interactive installation

## Recommended Fresh Install

### 1. Create the workspace and clone the repository

```bash
mkdir -p ~/nexus_swarm_sim_ws/src
cd ~/nexus_swarm_sim_ws/src
git clone https://github.com/tayfurcnr/nexus_swarm_sim.git
```

### 2. Run the setup script

```bash
cd ~/nexus_swarm_sim_ws/src/nexus_swarm_sim
bash setup_ardupilot_noetic.sh
```

Non-interactive variant:

```bash
bash setup_ardupilot_noetic.sh --yes
```

### 3. Source the workspace

```bash
cd ~/nexus_swarm_sim_ws
source devel/setup.bash
```

### 4. Add ArduPilot tools to PATH

```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=$PATH:$HOME/ardupilot/Tools
```

To persist this configuration:

```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=$PATH:$HOME/ardupilot/Tools' >> ~/.bashrc
source ~/.bashrc
```

## Pre-Launch Validation

Before starting the full stack, validate the expected runtime paths:

```bash
ls ~/ardupilot
ls ~/ardupilot/Tools/autotest/sim_vehicle.py
ls ~/ardupilot_gazebo
ls ~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf
rospack find mavros
```

If all of these commands succeed, the core `full_swarm.launch` prerequisites are present.

## Successful Install Checklist

A machine is considered ready for the full stack when all of the following are true:
- `~/ardupilot` exists
- `~/ardupilot/Tools/autotest/sim_vehicle.py` exists
- `~/ardupilot_gazebo` exists
- `~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf` exists
- `rospack find mavros` succeeds
- `source ~/nexus_swarm_sim_ws/devel/setup.bash` succeeds
- `roslaunch nexus_swarm_sim uwb_only.launch num_drones:=3` starts successfully
- `roslaunch nexus_swarm_sim single_vehicle_sitl.launch` starts successfully

## Recommended Bringup Sequence

For a new machine, validate the stack in layers rather than starting directly with the full swarm launch.

### 1. UWB-only smoke test

```bash
roslaunch nexus_swarm_sim uwb_only.launch num_drones:=3
```

Validates:
- catkin workspace build
- Gazebo startup
- world loading
- UWB simulator startup
- basic model spawning with the repository dummy model

### 2. Gazebo model-only test

```bash
roslaunch nexus_swarm_sim models_only.launch num_drones:=3
```

Validates:
- Gazebo visualization
- multi-model spawning
- UWB simulation with a real Gazebo vehicle model

### 3. Single-vehicle SITL test

```bash
roslaunch nexus_swarm_sim single_vehicle_sitl.launch
```

Validates:
- ArduPilot SITL
- `ardupilot_gazebo`
- MAVROS
- one-vehicle end-to-end integration

### 4. Full swarm test

```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3
```

Validates the intended full-stack swarm workflow.

## Dependency Recovery

### If `~/ardupilot` does not exist

Preferred recovery path:

```bash
bash setup_ardupilot_noetic.sh
```

Manual recovery path:

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.bashrc
./waf configure --board sitl
./waf copter
```

### If `~/ardupilot_gazebo` does not exist

Preferred recovery path:

```bash
bash setup_ardupilot_noetic.sh
```

Manual recovery path:

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ~/ardupilot_gazebo
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### If `sim_vehicle.py` is missing

Expected file:

```bash
~/ardupilot/Tools/autotest/sim_vehicle.py
```

Fix:
- ensure ArduPilot is cloned into `~/ardupilot`
- rebuild SITL if needed

### If `iris_with_ardupilot/model.sdf` is missing

Expected file:

```bash
~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf
```

Fix:
- ensure `ardupilot_gazebo` is cloned into `~/ardupilot_gazebo`
- rebuild the plugin and model repository if needed

### If `rospack find mavros` fails

```bash
sudo apt update
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geographiclib.sh
sudo bash /tmp/install_geographiclib.sh
```

### If Python dependencies are missing

```bash
cd ~/nexus_swarm_sim_ws
python3 -m pip install -r src/nexus_swarm_sim/requirements.txt
```

## Model Sources By Launch Mode

`uwb_only.launch`
- repository model: `models/sdf/uwb_dummy.sdf`

`models_only.launch`
- system Gazebo model: `/usr/share/gazebo-11/models/iris_with_standoffs/model.sdf`

`single_vehicle_sitl.launch` and `full_swarm.launch`
- ArduPilot Gazebo model: `~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf`

## Gimbal Asset Note

The SITL model spawner can look for `gimbal_small_2d` in common Gazebo model locations:
- `~/.gazebo/models/gimbal_small_2d`
- `~/Downloads/models/gimbal_small_2d`

If gimbal-related spawning fails, disable gimbal first:

```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 enable_gimbal:=false
```
