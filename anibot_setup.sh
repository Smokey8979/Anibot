#!/bin/bash
# =============================================================================
#  anibot_setup.sh
#  ROS2 Jazzy + Nav2 + SLAM dependency checker & installer
#  Ubuntu 24.04 | ROS2 Jazzy
#  Run with: chmod +x anibot_setup.sh && ./anibot_setup.sh
#  (No sudo needed — script calls sudo internally where required)
# =============================================================================

# NOTE: Do NOT use 'set -e' here — bash arithmetic ((var++)) returns exit 1
#       when the value is 0, which would kill the script on the first counter.

# ── Detect real user even when run with sudo ──────────────────────────────────
REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME=$(eval echo "~$REAL_USER")

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Counters (use += style to avoid bash arithmetic exit-code bug) ─────────────
INSTALLED=0
ALREADY_OK=0
FAILED=0
FAILED_LIST=()

inc_ok()        { ALREADY_OK=$((ALREADY_OK + 1)); }
inc_installed() { INSTALLED=$((INSTALLED + 1)); }
inc_failed()    { FAILED=$((FAILED + 1)); }

# ── Helpers ───────────────────────────────────────────────────────────────────
print_header() {
    echo ""
    echo -e "${BLUE}${BOLD}══════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}${BOLD}  $1${NC}"
    echo -e "${BLUE}${BOLD}══════════════════════════════════════════════════════${NC}"
}

print_section() {
    echo ""
    echo -e "${CYAN}${BOLD}▶ $1${NC}"
    echo -e "${CYAN}──────────────────────────────────────────────────────${NC}"
}

check_and_install() {
    local pkg="$1"
    local desc="$2"
    if dpkg -s "$pkg" &>/dev/null 2>&1; then
        echo -e "  ${GREEN}✔${NC}  $pkg${desc:+ (${desc})}"
        inc_ok
    else
        echo -ne "  ${YELLOW}↓${NC}  Installing $pkg${desc:+ (${desc})}... "
        if sudo apt-get install -y "$pkg" -qq 2>/dev/null; then
            echo -e "${GREEN}done${NC}"
            inc_installed
        else
            echo -e "${RED}FAILED${NC}"
            inc_failed
            FAILED_LIST+=("$pkg")
        fi
    fi
}

check_pip_and_install() {
    local pkg="$1"
    local import_name="${2:-$1}"
    if python3 -c "import $import_name" &>/dev/null 2>&1; then
        echo -e "  ${GREEN}✔${NC}  python3: $pkg"
        inc_ok
    else
        echo -ne "  ${YELLOW}↓${NC}  pip install $pkg... "
        if pip3 install "$pkg" --break-system-packages -q 2>/dev/null; then
            echo -e "${GREEN}done${NC}"
            inc_installed
        else
            echo -e "${RED}FAILED${NC}"
            inc_failed
            FAILED_LIST+=("pip:$pkg")
        fi
    fi
}

check_exec() {
    local cmd="$1"
    if command -v "$cmd" &>/dev/null 2>&1; then
        echo -e "  ${GREEN}✔${NC}  $cmd"
        inc_ok
    else
        echo -e "  ${RED}✘${NC}  $cmd — NOT FOUND"
        inc_failed
        FAILED_LIST+=("exec:$cmd")
    fi
}

# =============================================================================
print_header "ANIBOT Nav2 Dependency Setup — ROS2 Jazzy / Ubuntu 24.04"
echo ""
echo -e "  Date      : $(date)"
echo -e "  Real user : $REAL_USER"
echo -e "  Home      : $REAL_HOME"
echo -e "  Ubuntu    : $(lsb_release -rs 2>/dev/null || echo 'unknown')"
echo ""

# =============================================================================
print_section "0. Sanity Checks"

UBUNTU_VER=$(lsb_release -rs 2>/dev/null)
if [[ "$UBUNTU_VER" == "24.04" ]]; then
    echo -e "  ${GREEN}✔${NC}  Ubuntu 24.04 confirmed"
else
    echo -e "  ${YELLOW}⚠${NC}  Expected Ubuntu 24.04, got $UBUNTU_VER — proceeding anyway"
fi

if [ -d "/opt/ros/jazzy" ]; then
    echo -e "  ${GREEN}✔${NC}  ROS2 Jazzy found at /opt/ros/jazzy"
else
    echo -e "  ${RED}✘${NC}  ROS2 Jazzy NOT found. Attempting install..."
    sudo apt-get update -qq
    sudo apt-get install -y curl gnupg2 lsb-release -qq
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu noble main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y ros-jazzy-ros-base -qq
    echo -e "  ${GREEN}✔${NC}  ROS2 Jazzy installed"
    inc_installed
fi

echo -ne "  ${YELLOW}↓${NC}  Refreshing apt cache... "
sudo apt-get update -qq 2>/dev/null
echo -e "${GREEN}done${NC}"
inc_ok

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

# =============================================================================
print_section "1. System Build Tools & Essentials"

check_and_install "build-essential"                    "C/C++ build tools"
check_and_install "cmake"                              "CMake"
check_and_install "git"                                "version control"
check_and_install "python3-pip"                        "Python pip"
check_and_install "python3-dev"                        "Python headers"
check_and_install "python3-setuptools"                 "setuptools"
check_and_install "python3-colcon-common-extensions"   "colcon build tool"
check_and_install "python3-rosdep"                     "rosdep"
check_and_install "python3-vcstool"                    "vcs tool"
check_and_install "python3-argcomplete"                "shell completion"
check_and_install "libeigen3-dev"                      "Eigen math library"
check_and_install "libboost-all-dev"                   "Boost C++ libraries"

# =============================================================================
print_section "2. ROS2 Jazzy Core"

check_and_install "ros-jazzy-ros-base"                 "ROS2 base"
check_and_install "ros-jazzy-rclpy"                    "Python client"
check_and_install "ros-jazzy-rclcpp"                   "C++ client"
check_and_install "ros-jazzy-std-msgs"                 "std messages"
check_and_install "ros-jazzy-geometry-msgs"            "geometry messages"
check_and_install "ros-jazzy-nav-msgs"                 "nav messages"
check_and_install "ros-jazzy-sensor-msgs"              "sensor messages"
check_and_install "ros-jazzy-tf2"                      "TF2 core"
check_and_install "ros-jazzy-tf2-ros"                  "TF2 ROS"
check_and_install "ros-jazzy-tf2-tools"                "view_frames etc."
check_and_install "ros-jazzy-tf2-geometry-msgs"        "TF2 geometry helpers"
check_and_install "ros-jazzy-rosbag2"                  "bag recording"
check_and_install "ros-jazzy-ros2cli"                  "ros2 CLI"

# =============================================================================
print_section "3. Robot State Publisher & URDF Tools"

check_and_install "ros-jazzy-robot-state-publisher"    "URDF TF publisher"
check_and_install "ros-jazzy-joint-state-publisher"    "joint state publisher"
check_and_install "ros-jazzy-joint-state-publisher-gui" "joint state GUI"
check_and_install "ros-jazzy-xacro"                    "xacro processor"
check_and_install "ros-jazzy-urdf"                     "URDF library"
check_and_install "ros-jazzy-urdf-parser-plugin"       "URDF parser plugin"
check_and_install "liburdfdom-tools"                   "check_urdf CLI"

# =============================================================================
print_section "4. Nav2 Full Stack"

check_and_install "ros-jazzy-navigation2"              "Nav2 meta-package"
check_and_install "ros-jazzy-nav2-bringup"             "Nav2 launch files"
check_and_install "ros-jazzy-nav2-core"                "Nav2 core interfaces"
check_and_install "ros-jazzy-nav2-msgs"                "Nav2 message types"
check_and_install "ros-jazzy-nav2-common"              "Nav2 utilities"
check_and_install "ros-jazzy-nav2-map-server"          "map_server + map_saver"
check_and_install "ros-jazzy-nav2-amcl"                "AMCL localizer"
check_and_install "ros-jazzy-nav2-bt-navigator"        "BT navigator"
check_and_install "ros-jazzy-nav2-planner"             "global planner server"
check_and_install "ros-jazzy-nav2-controller"          "local controller server"
check_and_install "ros-jazzy-nav2-smoother"            "path smoother"
check_and_install "ros-jazzy-nav2-behaviors"           "recovery behaviors"
check_and_install "ros-jazzy-nav2-waypoint-follower"   "waypoint follower"
check_and_install "ros-jazzy-nav2-lifecycle-manager"   "lifecycle manager"
check_and_install "ros-jazzy-nav2-costmap-2d"          "2D costmap + plugins"
check_and_install "ros-jazzy-nav2-navfn-planner"       "NavFn / GridBased planner"
check_and_install "ros-jazzy-nav2-theta-star-planner"  "Theta* planner"
check_and_install "ros-jazzy-nav2-smac-planner"        "SMAC / hybrid-A* planner"
check_and_install "ros-jazzy-nav2-velocity-smoother"   "velocity smoother"
check_and_install "ros-jazzy-nav2-collision-monitor"   "collision monitor"
check_and_install "ros-jazzy-nav2-mppi-controller"     "MPPI controller"
check_and_install "ros-jazzy-nav2-regulated-pure-pursuit-controller" "RPP controller (anibot)"
check_and_install "ros-jazzy-nav2-rotation-shim-controller" "rotation shim"
check_and_install "ros-jazzy-nav2-simple-commander"    "Python Nav2 API"
check_and_install "ros-jazzy-nav2-util"                "Nav2 util library"

# =============================================================================
print_section "5. Behavior Tree Plugins"

check_and_install "ros-jazzy-behaviortree-cpp-v3"      "BehaviorTree.CPP v3"
check_and_install "ros-jazzy-nav2-behavior-tree"       "Nav2 BT node plugins"

# =============================================================================
print_section "6. SLAM Toolbox"

check_and_install "ros-jazzy-slam-toolbox"             "async SLAM mapping"

# =============================================================================
print_section "7. Localization & Map Tools"

check_and_install "ros-jazzy-map-msgs"                 "map message types"
check_and_install "ros-jazzy-pcl-ros"                  "PCL ROS"
check_and_install "ros-jazzy-laser-geometry"           "LaserScan to PointCloud"
check_and_install "ros-jazzy-filters"                  "sensor filter chain"

# =============================================================================
print_section "8. RViz2 & Nav2 RViz Plugins"

check_and_install "ros-jazzy-rviz2"                    "RViz2 visualizer"
check_and_install "ros-jazzy-rviz-common"              "RViz2 common libs"
check_and_install "ros-jazzy-rviz-default-plugins"     "RViz2 default plugins"
check_and_install "ros-jazzy-nav2-rviz-plugins"        "Nav2 RViz panels"

# =============================================================================
print_section "9. Serial & Python Dependencies (for new_odom.py)"

check_and_install "python3-serial"                     "pyserial apt package"
check_pip_and_install "pyserial" "serial"
check_pip_and_install "numpy"    "numpy"
check_pip_and_install "transforms3d" "transforms3d"

if groups "$REAL_USER" 2>/dev/null | grep -q dialout; then
    echo -e "  ${GREEN}✔${NC}  $REAL_USER is in dialout group (serial /dev/ttyACM0 OK)"
    inc_ok
else
    echo -ne "  ${YELLOW}↓${NC}  Adding $REAL_USER to dialout group... "
    sudo usermod -aG dialout "$REAL_USER"
    echo -e "${GREEN}done${NC}  ← log out & back in for this to take effect"
    inc_installed
fi

# =============================================================================
print_section "10. Teleop Tools"

check_and_install "ros-jazzy-teleop-twist-keyboard"    "keyboard teleop"
check_and_install "ros-jazzy-teleop-twist-joy"         "joystick teleop"
check_and_install "ros-jazzy-joy"                      "joystick driver"

# =============================================================================
print_section "11. rosdep Init & Update"

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo -ne "  ${YELLOW}↓${NC}  Initialising rosdep... "
    sudo rosdep init 2>/dev/null || true
    echo -e "${GREEN}done${NC}"
    inc_installed
else
    echo -e "  ${GREEN}✔${NC}  rosdep already initialised"
    inc_ok
fi

echo -ne "  ${YELLOW}↓${NC}  Updating rosdep... "
sudo -u "$REAL_USER" rosdep update --rosdistro jazzy -q 2>/dev/null || rosdep update --rosdistro jazzy -q 2>/dev/null || true
echo -e "${GREEN}done${NC}"
inc_ok

# =============================================================================
print_section "12. rosdep install on Your Workspace"

WS_DIR="$REAL_HOME/anibot_ws"
if [ -d "$WS_DIR/src" ]; then
    echo -e "  ${CYAN}ℹ${NC}  Running rosdep install on $WS_DIR/src ..."
    rosdep install --from-paths "$WS_DIR/src" \
        --ignore-src -r -y --rosdistro jazzy 2>&1 | \
        grep -v "^#" | sed 's/^/     /' || true
    echo -e "  ${GREEN}✔${NC}  rosdep install complete"
    inc_ok
else
    echo -e "  ${YELLOW}⚠${NC}  Workspace not found at $WS_DIR — skipping"
fi

# =============================================================================
print_section "13. Verify Key Executables"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

check_exec "ros2"
check_exec "colcon"
check_exec "rviz2"
check_exec "check_urdf"
check_exec "xacro"

# =============================================================================
print_section "14. ~/.bashrc Source Lines"

BASHRC="$REAL_HOME/.bashrc"
ROS_SOURCE="source /opt/ros/jazzy/setup.bash"
WS_SOURCE="source $REAL_HOME/anibot_ws/install/setup.bash"

if grep -qF "$ROS_SOURCE" "$BASHRC" 2>/dev/null; then
    echo -e "  ${GREEN}✔${NC}  ROS2 Jazzy already sourced in ~/.bashrc"
    inc_ok
else
    echo "$ROS_SOURCE" >> "$BASHRC"
    echo -e "  ${GREEN}✔${NC}  Added ROS2 Jazzy source to ~/.bashrc"
    inc_installed
fi

if grep -qF "anibot_ws/install/setup.bash" "$BASHRC" 2>/dev/null; then
    echo -e "  ${GREEN}✔${NC}  anibot_ws already sourced in ~/.bashrc"
    inc_ok
else
    cat >> "$BASHRC" <<EOF

# anibot_ws — added by anibot_setup.sh
if [ -f $REAL_HOME/anibot_ws/install/setup.bash ]; then
  $WS_SOURCE
fi
EOF
    echo -e "  ${GREEN}✔${NC}  Added anibot_ws source to ~/.bashrc"
    inc_installed
fi

# =============================================================================
print_header "SETUP COMPLETE — Summary"

echo ""
echo -e "  ${GREEN}✔  Already installed : $ALREADY_OK${NC}"
echo -e "  ${YELLOW}↓  Newly installed   : $INSTALLED${NC}"

if [ "${#FAILED_LIST[@]}" -gt 0 ]; then
    echo -e "  ${RED}✘  Failed             : $FAILED${NC}"
    echo ""
    echo -e "  ${RED}Failed packages:${NC}"
    for f in "${FAILED_LIST[@]}"; do
        echo -e "    ${RED}•${NC} $f"
    done
else
    echo -e "  ${GREEN}✔  Zero failures — everything installed!${NC}"
fi

echo ""
echo -e "${BOLD}▶ Next Steps:${NC}"
echo ""
echo -e "  1. ${CYAN}Rebuild    :${NC}  cd ~/anibot_ws && colcon build --symlink-install"
echo -e "  2. ${CYAN}Source     :${NC}  source install/setup.bash"
echo -e "  3. ${CYAN}Verify URDF:${NC}  check_urdf src/interface/urdf/chassis_fixed.urdf"
echo -e "  4. ${CYAN}Bringup    :${NC}  ros2 launch interface bringup.launch.py"
echo -e "  5. ${CYAN}SLAM       :${NC}  ros2 launch interface slam.launch.py"
echo -e "  6. ${CYAN}Teleop     :${NC}  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo -e "  7. ${CYAN}Save map   :${NC}  ros2 run nav2_map_server map_saver_cli -f ~/my_map"
echo -e "  8. ${CYAN}Nav2       :${NC}  ros2 launch interface nav2.launch.py map:=\$HOME/my_map.yaml"
echo -e "  9. ${CYAN}RViz2      :${NC}  ros2 launch nav2_bringup rviz_launch.py"
echo ""
if groups "$REAL_USER" 2>/dev/null | grep -qv dialout; then
    echo -e "  ${YELLOW}⚠  Remember to log out & back in for dialout group to apply!${NC}"
    echo ""
fi
echo -e "${GREEN}${BOLD}  Done! Happy navigating 🤖${NC}"
echo ""
