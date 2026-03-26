#!/bin/bash
# Swarm Gazebo Simulation - ArduPilot + ROS Noetic Setup
# This script prepares a catkin workspace for the full ArduPilot-backed test flow.

set -e

WORKSPACE_DIR="${WORKSPACE_DIR:-${HOME}/nexus_swarm_sim_ws}"
REPO_URL="https://github.com/tayfurcnr/nexus_swarm_sim.git"
REPO_DIR="${WORKSPACE_DIR}/src/nexus_swarm_sim"
ARDUPILOT_DIR="${HOME}/ardupilot"
ARDUPILOT_GAZEBO_DIR="${HOME}/ardupilot_gazebo"

echo "=================================================="
echo "Swarm Gazebo Simulation - ArduPilot Setup"
echo "Ubuntu 20.04 + ROS Noetic + Gazebo 11 Classic"
echo "Workspace: ${WORKSPACE_DIR}"
echo "=================================================="

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

AUTO_YES=0
if [ "${1:-}" = "--yes" ] || [ "${AUTO_YES:-0}" = "1" ]; then
    AUTO_YES=1
fi

confirm_step() {
    local step_label="$1"
    local title="$2"
    local target_path="$3"
    local summary="$4"
    local impact="$5"

    if [ "${AUTO_YES}" = "1" ]; then
        echo -e "${GREEN}✓${NC} auto-approved: ${step_label} ${title}"
        return 0
    fi

    echo ""
    echo -e "${YELLOW}[${step_label}] ${title}${NC}"
    echo "Target: ${target_path}"
    echo "Will do:"
    printf '  - %s\n' "${summary}"
    echo "If skipped:"
    printf '  - %s\n' "${impact}"
    echo "Options:"
    echo "  Y = continue with this step"
    echo "  N = skip this step"
    echo "  A = continue and auto-approve the remaining prompts"
    read -r -p "Proceed? [Y/N/A]: " reply
    case "${reply:-Y}" in
        a|A)
            AUTO_YES=1
            return 0
            ;;
        n|N)
            return 1
            ;;
        *)
            return 0
            ;;
    esac
}

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

if [ ! -d "${ARDUPILOT_DIR}" ]; then
    if confirm_step         "5A"         "Clone and build ArduPilot SITL"         "${ARDUPILOT_DIR}"         "clone ArduPilot, install SITL prerequisites, and build ArduCopter"         "full_swarm.launch and single_vehicle_sitl.launch will not work until ArduPilot is installed"; then
        echo -e "${YELLOW}→${NC} cloning ArduPilot"
        cd "${HOME}"
        git clone https://github.com/ArduPilot/ardupilot.git

        echo -e "${YELLOW}→${NC} installing ArduPilot prerequisites"
        cd "${ARDUPILOT_DIR}"
        sudo apt install -y python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml python3-pygame
        ./Tools/environment_install/install-prereqs-ubuntu.sh -y
        . "${HOME}/.bashrc"

        echo -e "${YELLOW}→${NC} building ArduCopter SITL (may take 10-15 minutes)"
        ./waf configure --board sitl > /dev/null 2>&1
        ./waf copter > /dev/null 2>&1
        echo -e "${GREEN}✓${NC} ArduPilot built"
    else
        echo -e "${YELLOW}!${NC} Skipped ArduPilot setup"
    fi
else
    echo -e "${GREEN}✓${NC} ArduPilot already present"
fi

if [ ! -d "${ARDUPILOT_GAZEBO_DIR}" ]; then
    if confirm_step         "5B"         "Clone and build ardupilot_gazebo"         "${ARDUPILOT_GAZEBO_DIR}"         "clone ardupilot_gazebo, build the Gazebo plugin, and install it system-wide"         "full_swarm.launch and single_vehicle_sitl.launch will not work until ardupilot_gazebo is installed"; then
        echo -e "${YELLOW}→${NC} cloning ardupilot_gazebo"
        cd "${HOME}"
        git clone https://github.com/ArduPilot/ardupilot_gazebo.git

        echo -e "${YELLOW}→${NC} building ardupilot_gazebo plugin"
        cd "${ARDUPILOT_GAZEBO_DIR}"
        mkdir -p build
        cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
        make -j"$(nproc)" > /dev/null 2>&1
        sudo make install > /dev/null 2>&1
        echo -e "${GREEN}✓${NC} ardupilot_gazebo installed"
    else
        echo -e "${YELLOW}!${NC} Skipped ardupilot_gazebo setup"
    fi
else
    echo -e "${GREEN}✓${NC} ardupilot_gazebo already present"
fi

echo -e "\n${YELLOW}[6/6] Catkin build${NC}"
cd "${WORKSPACE_DIR}"
catkin build > /dev/null 2>&1

echo -e "\n${GREEN}=================================================="
echo "✓ Setup complete"
echo -e "==================================================${NC}\n"

echo -e "${YELLOW}Next steps:${NC}"
echo "1. Source the workspace:"
echo "   cd ${WORKSPACE_DIR}"
echo "   source devel/setup.bash"
echo ""
echo "2. ArduPilot paths:"
echo "   ${ARDUPILOT_DIR}"
echo "   ${ARDUPILOT_GAZEBO_DIR}"
echo ""
echo "3. Add ArduPilot tools to PATH:"
echo "   export PATH=\$PATH:${ARDUPILOT_DIR}/Tools/autotest"
echo "   export PATH=\$PATH:${ARDUPILOT_DIR}/Tools"
echo ""
echo "4. Launch the full integration test with dashboard:"
echo "   roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus dashboard:=true dashboard_host:=0.0.0.0 dashboard_port:=8787"
echo ""
echo "5. Open the dashboard:"
echo "   http://localhost:8787"
echo ""
echo "   If you need the standalone demo dashboard without ROS topics:"
echo "   rosrun nexus_swarm_sim swarm_dashboard.py --demo --host 0.0.0.0 --port 8787"
echo ""
echo "   Note: uwb_only.launch is only for debug/smoke testing."
echo ""
echo "6. Monitor UWB topics from another terminal:"
echo "   rosrun nexus_swarm_sim swarm_uwb_monitor.py"
echo ""
echo -e "${YELLOW}Docs:${NC}"
echo "- README.md"
