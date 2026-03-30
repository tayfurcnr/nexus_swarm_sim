#!/bin/bash
# nexus_swarm_sim setup
# Prepares the ROS Noetic, Gazebo 11, and ArduPilot-backed workspace flow.

set -Eeuo pipefail

DEFAULT_WORKSPACE_DIR="${HOME}/swarm_ws"
WORKSPACE_DIR="${WORKSPACE_DIR:-}"
BUILD_TOOL="${BUILD_TOOL:-}"
DRY_RUN="${DRY_RUN:-0}"
FORCE_SWITCH_BUILD_TOOL="${FORCE_SWITCH_BUILD_TOOL:-0}"
REPO_URL="https://github.com/tayfurcnr/nexus_swarm_sim.git"
REPO_DIR=""
ARDUPILOT_DIR="${HOME}/ardupilot"
ARDUPILOT_GAZEBO_DIR="${HOME}/ardupilot_gazebo"
BASHRC_FILE="${HOME}/.bashrc"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

AUTO_YES=0
EXPLICIT_WORKSPACE=0
EXPLICIT_BUILD_TOOL=0
EXPLICIT_FORCE_SWITCH=0
expand_path() {
    local input="$1"
    case "${input}" in
        "~")
            printf '%s\n' "${HOME}"
            ;;
        "~/"*)
            printf '%s\n' "${HOME}/${input#~/}"
            ;;
        *)
            printf '%s\n' "${input}"
            ;;
    esac
}

refresh_workspace_paths() {
    WORKSPACE_DIR="$(expand_path "${WORKSPACE_DIR}")"
    REPO_DIR="${WORKSPACE_DIR}/src/nexus_swarm_sim"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --yes)
            AUTO_YES=1
            ;;
        --workspace)
            shift
            [ $# -gt 0 ] || {
                echo "Error: --workspace requires a path" >&2
                exit 1
            }
            WORKSPACE_DIR="$1"
            EXPLICIT_WORKSPACE=1
            ;;
        --build-tool)
            shift
            [ $# -gt 0 ] || {
                echo "Error: --build-tool requires a value" >&2
                exit 1
            }
            BUILD_TOOL="$1"
            EXPLICIT_BUILD_TOOL=1
            ;;
        --dry-run)
            DRY_RUN=1
            ;;
        --force-switch-build-tool)
            FORCE_SWITCH_BUILD_TOOL=1
            EXPLICIT_FORCE_SWITCH=1
            ;;
        *)
            echo "Error: Unknown argument: $1" >&2
            exit 1
            ;;
    esac
    shift
done

WORKSPACE_DIR="${WORKSPACE_DIR:-${DEFAULT_WORKSPACE_DIR}}"
BUILD_TOOL="${BUILD_TOOL:-auto}"
refresh_workspace_paths

if [ ! -t 0 ] && [ "${AUTO_YES}" = "0" ]; then
    AUTO_YES=1
fi

fail() {
    echo -e "${RED}✗${NC} $1" >&2
    exit 1
}

cancel_setup() {
    echo -e "${YELLOW}!${NC} Setup cancelled." >&2
    exit 0
}

info() {
    echo -e "${YELLOW}→${NC} $1"
}

success() {
    echo -e "${GREEN}✓${NC} $1"
}

warn() {
    echo -e "${YELLOW}!${NC} $1"
}

command_exists() {
    command -v "$1" >/dev/null 2>&1
}

run_cmd() {
    if [ "${DRY_RUN}" = "1" ]; then
        info "[dry-run] $*"
        return 0
    fi
    "$@"
}

run_quiet() {
    if [ "${DRY_RUN}" = "1" ]; then
        info "[dry-run] $*"
        return 0
    fi
    "$@" > /dev/null 2>&1
}

change_dir() {
    local target_dir="$1"
    if [ "${DRY_RUN}" = "1" ]; then
        info "[dry-run] cd ${target_dir}"
        return 0
    fi
    cd "${target_dir}"
}

on_error() {
    local exit_code=$?
    echo -e "\n${RED}✗ Setup failed${NC} at line ${BASH_LINENO[0]} while running: ${BASH_COMMAND}" >&2
    exit "${exit_code}"
}

trap on_error ERR

require_command() {
    command_exists "$1" || fail "Required command not found: $1"
}

package_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

append_line_once() {
    local line="$1"
    local file="$2"
    if [ "${DRY_RUN}" = "1" ]; then
        info "[dry-run] append to ${file}: ${line}"
        return 0
    fi
    touch "${file}"
    grep -Fqx "${line}" "${file}" 2>/dev/null || echo "${line}" >> "${file}"
}

cleanup_workspace_build_state() {
    if [ "${DRY_RUN}" = "1" ]; then
        info "[dry-run] rm -rf ${WORKSPACE_DIR}/build ${WORKSPACE_DIR}/devel ${WORKSPACE_DIR}/logs ${WORKSPACE_DIR}/.catkin_tools"
        return 0
    fi

    rm -rf \
        "${WORKSPACE_DIR}/build" \
        "${WORKSPACE_DIR}/devel" \
        "${WORKSPACE_DIR}/logs" \
        "${WORKSPACE_DIR}/.catkin_tools"
}

assert_supported_platform() {
    require_command lsb_release

    local distro release
    distro="$(lsb_release -is)"
    release="$(lsb_release -rs)"

    if [ "${distro}" != "Ubuntu" ] || [ "${release}" != "20.04" ]; then
        fail "This installer targets Ubuntu 20.04. Detected ${distro} ${release}."
    fi

    if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
        fail "ROS Noetic was not found at /opt/ros/noetic/setup.bash."
    fi
}

validate_installation() {
    local missing=0

    if [ "${DRY_RUN}" = "1" ]; then
        success "Dry run completed. Validation was skipped."
        return 0
    fi

    if [ ! -f "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" ]; then
        echo -e "${YELLOW}!${NC} Missing ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
        missing=1
    fi

    if [ ! -f "${ARDUPILOT_GAZEBO_DIR}/models/iris_with_ardupilot/model.sdf" ]; then
        echo -e "${YELLOW}!${NC} Missing ${ARDUPILOT_GAZEBO_DIR}/models/iris_with_ardupilot/model.sdf"
        missing=1
    fi

    if ! rospack find mavros >/dev/null 2>&1; then
        echo -e "${YELLOW}!${NC} rospack could not find mavros"
        missing=1
    fi

    if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
        echo -e "${YELLOW}!${NC} Missing ${WORKSPACE_DIR}/devel/setup.bash"
        missing=1
    fi

    if [ "${missing}" = "0" ]; then
        success "Post-install validation passed."
    else
        echo -e "${YELLOW}!${NC} Setup completed with validation warnings."
    fi
}

detect_build_tool() {
    case "${BUILD_TOOL}" in
        auto)
            if [ -d "${WORKSPACE_DIR}/.catkin_tools" ]; then
                echo "catkin_build"
                return 0
            fi

            if [ -f "${WORKSPACE_DIR}/build/CMakeCache.txt" ] || \
               [ -f "${WORKSPACE_DIR}/devel/.catkin" ] || \
               [ -f "${WORKSPACE_DIR}/build/Makefile" ]; then
                echo "catkin_make"
                return 0
            fi

            echo "catkin_build"
            ;;
        catkin_build|catkin_make)
            echo "${BUILD_TOOL}"
            ;;
        *)
            fail "Unsupported BUILD_TOOL='${BUILD_TOOL}'. Use auto, catkin_build, or catkin_make."
            ;;
    esac
}

workspace_looks_initialized() {
    [ -d "${WORKSPACE_DIR}/.catkin_tools" ] || \
    [ -f "${WORKSPACE_DIR}/build/CMakeCache.txt" ] || \
    [ -f "${WORKSPACE_DIR}/build/Makefile" ] || \
    [ -f "${WORKSPACE_DIR}/devel/.catkin" ]
}

use_whiptail() {
    [ "${AUTO_YES}" = "0" ] && [ -t 0 ] && command_exists whiptail
}

prompt_text_input() {
    local prompt="$1"
    local default_value="$2"
    local reply

    read -r -p "${prompt} [${default_value}]: " reply
    printf '%s\n' "${reply:-${default_value}}"
}

prompt_workspace_mode() {
    local choice

    if use_whiptail; then
        choice=$(whiptail --title "Workspace Selection" --menu \
            "Select the target workspace mode for nexus_swarm_sim." \
            15 72 2 \
            "new" "Create a new workspace" \
            "existing" "Use an existing workspace" \
            3>&1 1>&2 2>&3) || cancel_setup
        printf '%s\n' "${choice}"
        return 0
    fi

    echo ""
    echo "Workspace selection:"
    echo "  1) Create a new workspace"
    echo "  2) Use an existing workspace"
    while true; do
        read -r -p "Select workspace mode [1/2]: " choice
        case "${choice:-1}" in
            1) printf '%s\n' "new"; return 0 ;;
            2) printf '%s\n' "existing"; return 0 ;;
            *) echo "Enter 1 or 2." ;;
        esac
    done
}

prompt_workspace_path() {
    local mode="$1"
    local default_path="$2"
    local prompt_title prompt_text path

    if [ "${mode}" = "new" ]; then
        prompt_title="New Workspace"
        prompt_text="Enter the path for the new workspace."
    else
        prompt_title="Existing Workspace"
        prompt_text="Enter the path to the existing catkin workspace."
    fi

    if use_whiptail; then
        path=$(whiptail --title "${prompt_title}" --inputbox \
            "${prompt_text}" \
            11 72 "${default_path}" \
            3>&1 1>&2 2>&3) || cancel_setup
        printf '%s\n' "${path}"
        return 0
    fi

    prompt_text_input "Workspace path" "${default_path}"
}

prompt_build_tool_for_new_workspace() {
    local choice

    if use_whiptail; then
        choice=$(whiptail --title "Build Tool Selection" --menu \
            "Select the build tool for this workspace." \
            15 72 2 \
            "catkin_build" "catkin build (recommended)" \
            "catkin_make" "catkin_make" \
            3>&1 1>&2 2>&3) || cancel_setup
        printf '%s\n' "${choice}"
        return 0
    fi

    echo ""
    echo "Build tool selection:"
    echo "  1) catkin build (recommended)"
    echo "  2) catkin_make"
    while true; do
        read -r -p "Select build tool [1/2]: " choice
        case "${choice:-1}" in
            1) printf '%s\n' "catkin_build"; return 0 ;;
            2) printf '%s\n' "catkin_make"; return 0 ;;
            *) echo "Enter 1 or 2." ;;
        esac
    done
}

confirm_detected_build_tool() {
    local detected_tool="$1"
    local alternate_tool choice

    if [ "${detected_tool}" = "catkin_build" ]; then
        alternate_tool="catkin_make"
    else
        alternate_tool="catkin_build"
    fi

    if use_whiptail; then
        choice=$(whiptail --title "Detected Build Tool" --menu \
            "An existing workspace was detected. Select the build tool to use." \
            16 78 2 \
            "${detected_tool}" "Use detected tool" \
            "${alternate_tool}" "Override detected tool" \
            3>&1 1>&2 2>&3) || cancel_setup
        printf '%s\n' "${choice}"
        return 0
    fi

    echo ""
    echo "Detected build tool: ${detected_tool}"
    echo "  1) Use detected tool (${detected_tool})"
    echo "  2) Override to ${alternate_tool}"
    while true; do
        read -r -p "Select build tool [1/2]: " choice
        case "${choice:-1}" in
            1) printf '%s\n' "${detected_tool}"; return 0 ;;
            2) printf '%s\n' "${alternate_tool}"; return 0 ;;
            *) echo "Enter 1 or 2." ;;
        esac
    done
}

confirm_build_tool_switch() {
    local current_tool="$1"
    local requested_tool="$2"
    local choice

    if [ "${FORCE_SWITCH_BUILD_TOOL}" = "1" ]; then
        return 0
    fi

    if [ "${AUTO_YES}" = "1" ]; then
        return 1
    fi

    if use_whiptail; then
        whiptail --title "Switch Build Tool" --yesno \
            "The workspace currently uses ${current_tool}. Switching to ${requested_tool} requires removing build/, devel/, logs/, and .catkin_tools/ before rebuilding. Source files in src/ will be kept.\n\nDo you want to clean the workspace build state and continue?" \
            16 78
        return $?
    fi

    echo ""
    echo "Build tool switch required:"
    echo "  Current: ${current_tool}"
    echo "  Requested: ${requested_tool}"
    echo "  This will remove build/, devel/, logs/, and .catkin_tools/."
    echo "  Source files in src/ will be kept."
    read -r -p "Clean the workspace build state and continue? [y/N]: " choice
    case "${choice:-N}" in
        y|Y)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

show_completion_summary() {
    local selected_build_tool="$1"

    if use_whiptail; then
        whiptail --title "Installation Complete" --msgbox \
            "nexus_swarm_sim was installed successfully.

Workspace: ${WORKSPACE_DIR}
Build tool: ${selected_build_tool}

If you open a new terminal, the workspace and ArduPilot tool paths are already configured in ~/.bashrc.

For the current terminal, run:
source ${WORKSPACE_DIR}/devel/setup.bash" \
            18 90
    fi
}

configure_workspace_interactive() {
    local mode selected_tool workspace_exists workspace_has_src

    [ "${AUTO_YES}" = "0" ] || return 0
    [ -t 0 ] || return 0
    [ "${EXPLICIT_WORKSPACE}" = "0" ] || [ "${EXPLICIT_BUILD_TOOL}" = "0" ] || return 0

    if [ "${EXPLICIT_WORKSPACE}" = "1" ]; then
        refresh_workspace_paths
        workspace_exists=0
        workspace_has_src=0
        [ -d "${WORKSPACE_DIR}" ] && workspace_exists=1
        [ -d "${WORKSPACE_DIR}/src" ] && workspace_has_src=1

        if [ "${EXPLICIT_BUILD_TOOL}" = "0" ]; then
            if [ "${workspace_exists}" = "1" ] && [ "${workspace_has_src}" = "1" ] && workspace_looks_initialized; then
                selected_tool="$(detect_build_tool)"
                BUILD_TOOL="$(confirm_detected_build_tool "${selected_tool}")"
            else
                BUILD_TOOL="$(prompt_build_tool_for_new_workspace)"
            fi
        fi
        refresh_workspace_paths
        return 0
    fi

    mode="$(prompt_workspace_mode)"
    if [ "${mode}" = "new" ]; then
        WORKSPACE_DIR="$(prompt_workspace_path "new" "${WORKSPACE_DIR:-${DEFAULT_WORKSPACE_DIR}}")"
        if [ "${EXPLICIT_BUILD_TOOL}" = "0" ]; then
            BUILD_TOOL="$(prompt_build_tool_for_new_workspace)"
        fi
    else
        WORKSPACE_DIR="$(prompt_workspace_path "existing" "${WORKSPACE_DIR:-${DEFAULT_WORKSPACE_DIR}}")"
        refresh_workspace_paths
        [ -d "${WORKSPACE_DIR}" ] || fail "Existing workspace path does not exist: ${WORKSPACE_DIR}"
        [ -d "${WORKSPACE_DIR}/src" ] || fail "Existing workspace path is missing src/: ${WORKSPACE_DIR}/src"

        if [ "${EXPLICIT_BUILD_TOOL}" = "1" ]; then
            :
        elif workspace_looks_initialized; then
            selected_tool="$(detect_build_tool)"
            BUILD_TOOL="$(confirm_detected_build_tool "${selected_tool}")"
        else
            BUILD_TOOL="$(prompt_build_tool_for_new_workspace)"
        fi
    fi

    refresh_workspace_paths
}

run_workspace_build() {
    local selected_tool="$1"
    local current_tool

    case "${selected_tool}" in
        catkin_build)
            if [ -f "${WORKSPACE_DIR}/build/CMakeCache.txt" ] || \
               [ -f "${WORKSPACE_DIR}/build/Makefile" ]; then
                if [ "${DRY_RUN}" = "1" ]; then
                    if [ "${FORCE_SWITCH_BUILD_TOOL}" = "1" ]; then
                        warn "This workspace appears to contain catkin_make build artifacts. A real run would clean build/, devel/, logs/, and .catkin_tools/ before switching to catkin_build."
                    else
                        warn "This workspace appears to contain catkin_make build artifacts. A real run would fail unless you clean build/devel or use BUILD_TOOL=catkin_make."
                    fi
                    return 0
                fi
                current_tool="catkin_make"
                if confirm_build_tool_switch "${current_tool}" "catkin_build"; then
                    info "Cleaning workspace build state before switching to catkin_build"
                    cleanup_workspace_build_state
                else
                    fail "This workspace appears to contain catkin_make build artifacts. Clean build/devel or rerun with BUILD_TOOL=catkin_make."
                fi
            fi

            require_command catkin
            info "Building the workspace with catkin build"
            run_quiet catkin build
            ;;
        catkin_make)
            if [ -d "${WORKSPACE_DIR}/.catkin_tools" ]; then
                if [ "${DRY_RUN}" = "1" ]; then
                    if [ "${FORCE_SWITCH_BUILD_TOOL}" = "1" ]; then
                        warn "This workspace appears to contain catkin build metadata. A real run would clean build/, devel/, logs/, and .catkin_tools/ before switching to catkin_make."
                    else
                        warn "This workspace appears to contain catkin build metadata. A real run would fail unless you clean build/devel/.catkin_tools or use BUILD_TOOL=catkin_build."
                    fi
                    return 0
                fi
                current_tool="catkin_build"
                if confirm_build_tool_switch "${current_tool}" "catkin_make"; then
                    info "Cleaning workspace build state before switching to catkin_make"
                    cleanup_workspace_build_state
                else
                    fail "This workspace appears to contain catkin build metadata. Clean build/devel/.catkin_tools or rerun with BUILD_TOOL=catkin_build."
                fi
            fi

            require_command catkin_make
            info "Building the workspace with catkin_make"
            run_quiet catkin_make
            ;;
        *)
            fail "Internal error: unknown build tool '${selected_tool}'"
            ;;
    esac
}

confirm_step() {
    local step_label="$1"
    local title="$2"
    local target_path="$3"
    local summary="$4"
    local impact="$5"
    local choice

    if [ "${AUTO_YES}" = "1" ]; then
        echo -e "${GREEN}✓${NC} Auto-approved: ${step_label} ${title}"
        return 0
    fi

    if use_whiptail; then
        choice=$(whiptail --title "[${step_label}] ${title}" --menu \
            "Target: ${target_path}

Planned action:
- ${summary}

If skipped:
- ${impact}" \
            18 78 2 \
            "continue" "Run this step" \
            "skip" "Skip this step" \
            3>&1 1>&2 2>&3) || choice="skip"

        case "${choice}" in
            skip)
                return 1
                ;;
            *)
                return 0
                ;;
        esac
    fi

    echo ""
    echo -e "${YELLOW}[${step_label}] ${title}${NC}"
    echo "Target: ${target_path}"
    echo "Planned action:"
    printf '  - %s\n' "${summary}"
    echo "If skipped:"
    printf '  - %s\n' "${impact}"
    echo "Options:"
    echo "  Y = continue"
    echo "  N = skip"
    read -r -p "Proceed? [Y/N]: " reply
    case "${reply:-Y}" in
        n|N)
            return 1
            ;;
        *)
            return 0
            ;;
    esac
}

configure_workspace_interactive

echo "=================================================="
echo "nexus_swarm_sim Setup"
echo "ROS Noetic | Gazebo 11 Classic | ArduPilot SITL"
echo "Workspace: ${WORKSPACE_DIR}"
echo "Build Tool Preference: ${BUILD_TOOL}"
echo "Dry run: ${DRY_RUN}"
echo "=================================================="

assert_supported_platform
require_command sudo
require_command git
require_command wget
require_command python3
require_command dpkg-query
require_command rospack

echo -e "${YELLOW}[1/6] System packages${NC}"
run_cmd sudo apt update

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
    if package_installed "${pkg}"; then
        success "${pkg} is already installed"
    else
        info "Installing ${pkg}"
        run_cmd sudo apt install -y "${pkg}"
    fi
done

echo -e "\n${YELLOW}[2/6] GeographicLib datasets${NC}"
if [ ! -f "/etc/profile.d/mavros_geod.sh" ]; then
    run_quiet wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geographiclib.sh
    run_cmd sudo bash /tmp/install_geographiclib.sh
    success "GeographicLib datasets installed"
else
    success "GeographicLib datasets are already installed"
fi

echo -e "\n${YELLOW}[3/6] Workspace preparation${NC}"
run_cmd mkdir -p "${WORKSPACE_DIR}/src"
change_dir "${WORKSPACE_DIR}"

echo -e "\n${YELLOW}[4/6] Repository checkout${NC}"
if [ ! -d "${REPO_DIR}" ]; then
    info "Cloning nexus_swarm_sim"
    run_cmd git clone "${REPO_URL}" "${REPO_DIR}"
else
    success "nexus_swarm_sim is already present"
fi

echo -e "\n${YELLOW}[5/6] Python dependencies${NC}"
run_quiet sudo -H python3 -m pip install -q -r "${REPO_DIR}/requirements.txt"
success "Python dependencies installed"

if [ ! -d "${ARDUPILOT_DIR}" ]; then
    if confirm_step \
        "5A" \
        "Install ArduPilot SITL" \
        "${ARDUPILOT_DIR}" \
        "Clone ArduPilot, install SITL prerequisites, and build ArduCopter" \
        "full_swarm.launch and single_vehicle_sitl.launch will remain unavailable until ArduPilot is installed"; then
        info "Cloning ArduPilot"
        change_dir "${HOME}"
        run_cmd git clone https://github.com/ArduPilot/ardupilot.git

        info "Installing ArduPilot prerequisites"
        change_dir "${ARDUPILOT_DIR}"
        run_cmd git submodule update --init --recursive
        run_cmd sudo apt install -y python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml python3-pygame
        run_cmd ./Tools/environment_install/install-prereqs-ubuntu.sh -y
        if [ "${DRY_RUN}" = "0" ] && [ -f "${BASHRC_FILE}" ]; then
            # The installer may update shell startup files; reload them for this shell.
            . "${BASHRC_FILE}"
        fi

        info "Building ArduCopter SITL. This may take 10 to 15 minutes."
        run_quiet ./waf configure --board sitl
        run_quiet ./waf copter
        success "ArduPilot built successfully"
    else
        echo -e "${YELLOW}!${NC} Skipped ArduPilot installation"
    fi
else
    success "ArduPilot is already present"
fi

if [ ! -d "${ARDUPILOT_GAZEBO_DIR}" ]; then
    if confirm_step \
        "5B" \
        "Install ardupilot_gazebo" \
        "${ARDUPILOT_GAZEBO_DIR}" \
        "Clone ardupilot_gazebo, build the Gazebo plugin, and install it system-wide" \
        "full_swarm.launch and single_vehicle_sitl.launch will remain unavailable until ardupilot_gazebo is installed"; then
        info "Cloning ardupilot_gazebo"
        change_dir "${HOME}"
        run_cmd git clone https://github.com/ArduPilot/ardupilot_gazebo.git

        info "Building the ardupilot_gazebo plugin"
        change_dir "${ARDUPILOT_GAZEBO_DIR}"
        run_cmd mkdir -p build
        change_dir build
        run_quiet cmake .. -DCMAKE_BUILD_TYPE=Release
        run_quiet make -j"$(nproc)"
        run_quiet sudo make install
        success "ardupilot_gazebo installed successfully"
    else
        echo -e "${YELLOW}!${NC} Skipped ardupilot_gazebo installation"
    fi
else
    success "ardupilot_gazebo is already present"
fi

echo -e "\n${YELLOW}[6/6] Workspace build${NC}"
change_dir "${WORKSPACE_DIR}"
SELECTED_BUILD_TOOL="$(detect_build_tool)"
info "Selected build tool: ${SELECTED_BUILD_TOOL}"
run_workspace_build "${SELECTED_BUILD_TOOL}"

validate_installation

if confirm_step \
    "6A" \
    "Persist shell environment settings" \
    "${BASHRC_FILE}" \
    "Add workspace sourcing and ArduPilot PATH exports to ~/.bashrc if they are missing" \
    "You will need to source the workspace and export the ArduPilot tool paths manually in each new shell"; then
    append_line_once "source \"${WORKSPACE_DIR}/devel/setup.bash\"" "${BASHRC_FILE}"
    append_line_once "export PATH=\"\$PATH:${ARDUPILOT_DIR}/Tools/autotest\"" "${BASHRC_FILE}"
    append_line_once "export PATH=\"\$PATH:${ARDUPILOT_DIR}/Tools\"" "${BASHRC_FILE}"
    success "Shell environment entries ensured in ${BASHRC_FILE}"
else
    echo -e "${YELLOW}!${NC} Skipped shell environment persistence"
fi

echo -e "\n${GREEN}=================================================="
echo "Installation completed successfully."
echo -e "==================================================${NC}\n"

show_completion_summary "${SELECTED_BUILD_TOOL}"

echo -e "${YELLOW}Next steps:${NC}"
echo "1. In the current terminal, load the workspace:"
echo "   cd ${WORKSPACE_DIR}"
echo "   source devel/setup.bash"
echo ""
echo "2. Start the full integration test with the dashboard:"
echo "   roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus dashboard:=true dashboard_host:=0.0.0.0 dashboard_port:=8787"
echo ""
echo "3. Open the dashboard:"
echo "   http://localhost:8787"
echo ""
echo "   Optional standalone demo dashboard:"
echo "   rosrun nexus_swarm_sim swarm_dashboard.py --demo --host 0.0.0.0 --port 8787"
echo ""
echo "   Note: uwb_only.launch is intended only for debug and smoke testing."
echo ""
echo "4. Monitor UWB topics from another terminal:"
echo "   rosrun nexus_swarm_sim swarm_uwb_monitor.py"
echo ""
echo -e "${YELLOW}Docs:${NC}"
echo "- README.md"
