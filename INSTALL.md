# Installation Guide

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

![Nexus Swarm Sim Logo](docs/logo.png)

This guide describes the recommended installation, validation, and recovery flow for `nexus_swarm_sim`.

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
~/swarm_ws/
  src/
    nexus_swarm_sim/

~/ardupilot/
~/ardupilot_gazebo/
```

This layout matches the current runtime expectations used by the launch files and helper scripts.

## What The Setup Script Installs

Running [setup.sh](setup.sh) prepares the following:
- ROS, Gazebo, and MAVROS-related system packages
- GeographicLib datasets required by MAVROS
- Python dependencies from `requirements.txt`
- `~/ardupilot` if missing
- `~/ardupilot_gazebo` if missing
- the catkin workspace build for `nexus_swarm_sim`

The setup script also supports:
- interactive approval for long-running dependency steps
- `--yes` for unattended or non-interactive installation
- `WORKSPACE_DIR=/path/to/ws` to target an existing catkin workspace
- `BUILD_TOOL=auto|catkin_build|catkin_make` to control the workspace build step
- `--workspace /path/to/ws` and `--build-tool catkin_build|catkin_make|auto` as CLI flag equivalents
- `--dry-run` to preview the selected flow and commands without making changes
- `--force-switch-build-tool` to migrate an existing workspace to the selected build tool after cleaning old build metadata

## Recommended Fresh Install

### 1. Create the workspace and clone the repository

```bash
mkdir -p ~/swarm_ws/src
cd ~/swarm_ws/src
git clone https://github.com/tayfurcnr/nexus_swarm_sim.git
```

### 2. Run the setup script

```bash
cd ~/swarm_ws/src/nexus_swarm_sim
bash setup.sh
```

Non-interactive variant:

```bash
bash setup.sh --yes
```

Interactive mode asks:
- create a new workspace or use an existing one
- the target workspace path
- which build tool to use for a new workspace
- whether to keep or override the detected build tool for an existing workspace

If `whiptail` is available, the installer uses button-based dialogs for workspace selection, step approvals, build-tool migration confirmation, and the final success summary.

For a fresh workspace, the default build tool is `catkin build`. If the target workspace already contains `catkin_make` or `catkin build` metadata, the script detects that and offers to use the matching tool automatically.

### 2A. Existing workspace variant

If you already have a workspace and want to add `nexus_swarm_sim` into it:

```bash
cd /path/to/your_ws/src
git clone https://github.com/tayfurcnr/nexus_swarm_sim.git
cd nexus_swarm_sim
WORKSPACE_DIR=/path/to/your_ws bash setup.sh
```

If you need to force the build tool explicitly:

```bash
WORKSPACE_DIR=/path/to/your_ws BUILD_TOOL=catkin_make bash setup.sh
WORKSPACE_DIR=/path/to/your_ws BUILD_TOOL=catkin_build bash setup.sh
```

The same flow is also available with CLI flags:

```bash
bash setup.sh --workspace /path/to/your_ws --build-tool catkin_make
bash setup.sh --workspace /path/to/your_ws --build-tool catkin_build --yes
```

When `--workspace /path/to/your_ws` is provided, the script uses that path directly. If the path already contains a catkin workspace with `src/`, it is treated as an existing workspace. If the path does not exist yet, the script initializes a new workspace there.

To intentionally switch an existing workspace from one build system to the other:

```bash
bash setup.sh --workspace /path/to/your_ws --build-tool catkin_make --force-switch-build-tool
bash setup.sh --workspace /path/to/your_ws --build-tool catkin_build --force-switch-build-tool
```

This removes `build/`, `devel/`, `logs/`, and `.catkin_tools/` before rebuilding. `src/` is preserved.

Dry-run preview examples:

```bash
bash setup.sh --dry-run
bash setup.sh --workspace /path/to/your_ws --build-tool catkin_make --dry-run
```

### 3. Load the environment

If you accepted the shell persistence step, new terminals are already configured through `~/.bashrc`.

For the current terminal, run:

```bash
cd ~/swarm_ws
source devel/setup.bash
```

If you skipped shell persistence, also export the ArduPilot tool paths manually:

```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=$PATH:$HOME/ardupilot/Tools
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

If all of these commands succeed, the core prerequisites for `full_swarm.launch` are present.

## Successful Install Checklist

A machine is considered ready for the full stack when all of the following are true:
- `~/ardupilot` exists
- `~/ardupilot/Tools/autotest/sim_vehicle.py` exists
- `~/ardupilot_gazebo` exists
- `~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf` exists
- `rospack find mavros` succeeds
- `source ~/swarm_ws/devel/setup.bash` succeeds
- `roslaunch nexus_swarm_sim uwb_only.launch num_drones:=3` starts successfully
- `roslaunch nexus_swarm_sim single_vehicle_sitl.launch` starts successfully

## Recommended Bringup Sequence

On a new machine, validate the stack in layers rather than starting directly with the full swarm launch.

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
cd ~/swarm_ws/src/nexus_swarm_sim
bash setup.sh
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
cd ~/swarm_ws/src/nexus_swarm_sim
bash setup.sh
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
cd ~/swarm_ws
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
