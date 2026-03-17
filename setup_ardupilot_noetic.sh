#!/bin/bash
# Swarm Gazebo Simulation - ArduPilot + ROS Noetic Setup
# This script prepares a catkin workspace for the full ArduPilot-backed test flow.

set -e

WORKSPACE_DIR="${WORKSPACE_DIR:-${HOME}/nexus_swarm_sim_ws}"
REPO_URL="https://github.com/tayfurcnr/nexus_swarm_sim.git"
REPO_DIR="${WORKSPACE_DIR}/src/nexus_swarm_sim"

echo "=================================================="
echo "Swarm Gazebo Simulation - ArduPilot Setup"
echo "Ubuntu 20.04 + ROS Noetic + Gazebo 11 Classic"
echo "Workspace: ${WORKSPACE_DIR}"
echo "=================================================="

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}[1/6] System packages${NC}"
sudo apt update

PACKAGES=(
    "ros-noetic-mavros"
    "ros-noetic-mavros-extras"
    "ros-noetic-gazebo-ros"
    "ros-noetic-gazebo-plugins"
    "python3-catkin-tools"
    "python3-pip"
    "python3-dev"
    "git"
    "build-essential"
    "cmake"
    "wget"
)

for pkg in "${PACKAGES[@]}"; do
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "${GREEN}✓${NC} $pkg installed"
    else
        echo -e "${YELLOW}→${NC} installing $pkg"
        sudo apt install -y "$pkg"
    fi
done

echo -e "\n${YELLOW}[2/6] GeographicLib datasets${NC}"
if [ ! -f "/etc/profile.d/mavros_geod.sh" ]; then
    wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geographiclib.sh
    sudo bash /tmp/install_geographiclib.sh
    echo -e "${GREEN}✓${NC} GeographicLib datasets installed"
else
    echo -e "${GREEN}✓${NC} GeographicLib datasets already installed"
fi

echo -e "\n${YELLOW}[3/6] Workspace setup${NC}"
mkdir -p "${WORKSPACE_DIR}/src"
cd "${WORKSPACE_DIR}"

echo -e "\n${YELLOW}[4/6] Source repositories${NC}"
if [ ! -d "${REPO_DIR}" ]; then
    echo -e "${YELLOW}→${NC} cloning nexus_swarm_sim"
    git clone "${REPO_URL}" "${REPO_DIR}"
else
    echo -e "${GREEN}✓${NC} nexus_swarm_sim already present"
fi

echo -e "\n${YELLOW}[5/6] Python dependencies${NC}"
sudo -H python3 -m pip install -q -r "${REPO_DIR}/requirements.txt"
echo -e "${GREEN}✓${NC} Python dependencies installed"

if [ ! -d "${WORKSPACE_DIR}/src/ardupilot" ]; then
    echo -e "${YELLOW}→${NC} cloning ArduPilot"
    cd "${WORKSPACE_DIR}/src"
    git clone https://github.com/ArduPilot/ardupilot.git

    echo -e "${YELLOW}→${NC} installing ArduPilot prerequisites"
    cd "${WORKSPACE_DIR}/src/ardupilot"
    sudo apt install -y python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml python3-pygame
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y
    . "${HOME}/.bashrc"

    echo -e "${YELLOW}→${NC} building ArduCopter SITL (may take 10-15 minutes)"
    ./waf configure --board sitl > /dev/null 2>&1
    ./waf copter > /dev/null 2>&1
    echo -e "${GREEN}✓${NC} ArduPilot built"
else
    echo -e "${GREEN}✓${NC} ArduPilot already present"
fi

if [ ! -d "${WORKSPACE_DIR}/src/ardupilot_gazebo" ]; then
    echo -e "${YELLOW}→${NC} cloning ardupilot_gazebo"
    cd "${WORKSPACE_DIR}/src"
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git

    echo -e "${YELLOW}→${NC} building ardupilot_gazebo plugin"
    cd "${WORKSPACE_DIR}/src/ardupilot_gazebo"
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
    make -j"$(nproc)" > /dev/null 2>&1
    sudo make install > /dev/null 2>&1
    echo -e "${GREEN}✓${NC} ardupilot_gazebo installed"
else
    echo -e "${GREEN}✓${NC} ardupilot_gazebo already present"
fi

echo -e "\n${YELLOW}[6/6] Catkin build${NC}"
cd "${WORKSPACE_DIR}"
catkin build > /dev/null 2>&1

echo -e "\n${GREEN}=================================================="
echo "✓ Setup complete"
echo "==================================================${NC}\n"

echo -e "${YELLOW}Next steps:${NC}"
echo "1. Source the workspace:"
echo "   cd ${WORKSPACE_DIR}"
echo "   source devel/setup.bash"
echo ""
echo "2. Add ArduPilot tools to PATH:"
echo "   export PATH=\$PATH:${WORKSPACE_DIR}/src/ardupilot/Tools/autotest"
echo "   export PATH=\$PATH:${WORKSPACE_DIR}/src/ardupilot/Tools"
echo ""
echo "3. Launch the full integration test:"
echo "   roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus"
echo ""
echo "   Note: uwb_only.launch is only for debug/smoke testing."
echo ""
echo "4. Monitor UWB topics from another terminal:"
echo "   rosrun nexus_swarm_sim swarm_uwb_monitor.py"
echo ""
echo -e "${YELLOW}Docs:${NC}"
echo "- README.md"
