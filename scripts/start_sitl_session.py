#!/usr/bin/env python3

import os
import signal
import shutil
import subprocess
import sys
import termios
import time

import rospy


def prepend_env_path(name, value):
    current = os.environ.get(name, "")
    if current:
        os.environ[name] = f"{value}:{current}"
    else:
        os.environ[name] = value


def terminate_process_tree(process, sig=signal.SIGTERM):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), sig)
    except ProcessLookupError:
        pass


def capture_terminal_state():
    if not sys.stdin.isatty():
        return None
    try:
        return termios.tcgetattr(sys.stdin.fileno())
    except termios.error:
        return None


def restore_terminal_state(state):
    if state is None or not sys.stdin.isatty():
        return
    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, state)
    except termios.error:
        pass


def configure_ritw_environment(env, enable_console):
    if not enable_console:
        return

    # Route sim_vehicle's interactive helpers into a separate PTY so MAVProxy
    # does not continue writing into the parent shell after roslaunch exits.
    if shutil.which("tmux"):
        env["SITL_RITW_TERMINAL"] = "tmux new-window -dn"
    elif shutil.which("screen"):
        env["SITL_RITW_TERMINAL"] = "screen -D -m"
    elif shutil.which("xterm") and env.get("DISPLAY"):
        env["SITL_RITW_TERMINAL"] = "xterm -hold -e"


def cleanup_console_helpers(instance):
    sitl_tcp_port = 5760 + instance * 10
    mavproxy_out_port = 14550 + instance * 10
    sitl_listen_port = 5501 + instance * 10
    patterns = [
        f"mavproxy.py.*tcp:127.0.0.1:{sitl_tcp_port}",
        f"mavproxy.py.*127.0.0.1:{mavproxy_out_port}",
        f"mavproxy.py.*127.0.0.1:{sitl_listen_port}",
        f"screen .*tcp:127.0.0.1:{sitl_tcp_port}",
    ]

    for signal_name in ("-TERM", "-KILL"):
        for pattern in patterns:
            subprocess.run(["pkill", signal_name, "-f", pattern], check=False)
        time.sleep(0.2)


def main():
    rospy.init_node("sitl_session_launcher", anonymous=False)

    ardupilot_root = os.path.expanduser(rospy.get_param("~ardupilot_root", "~/ardupilot"))
    ardupilot_gazebo_root = os.path.expanduser(
        rospy.get_param("~ardupilot_gazebo_root", "~/ardupilot_gazebo")
    )
    vehicle = rospy.get_param("~vehicle", "ArduCopter")
    frame = rospy.get_param("~frame", "gazebo-iris")
    instance = int(rospy.get_param("~instance", 0))
    sysid = int(rospy.get_param("~sysid", 1))
    use_dir = rospy.get_param("~use_dir", "/tmp/ardupilot_sitl_single_vehicle")
    custom_location = rospy.get_param("~custom_location", "")
    add_param_file = rospy.get_param("~add_param_file", "")
    enable_mavproxy = bool(rospy.get_param("~enable_mavproxy", False))
    mavproxy_console = bool(rospy.get_param("~mavproxy_console", False))
    mavproxy_map = bool(rospy.get_param("~mavproxy_map", False))

    sim_vehicle = os.path.join(ardupilot_root, "Tools", "autotest", "sim_vehicle.py")
    if not os.path.isfile(sim_vehicle):
        rospy.logfatal("sim_vehicle.py not found at %s", sim_vehicle)
        return 1

    prepend_env_path("GAZEBO_MODEL_PATH", os.path.join(ardupilot_gazebo_root, "models"))
    prepend_env_path("GAZEBO_RESOURCE_PATH", os.path.join(ardupilot_gazebo_root, "worlds"))
    prepend_env_path("GAZEBO_PLUGIN_PATH", os.path.join(ardupilot_gazebo_root, "build"))
    prepend_env_path("PATH", os.path.join(ardupilot_root, "Tools", "autotest"))

    cmd = [
        sim_vehicle,
        "-v",
        vehicle,
        "-f",
        frame,
        "-I",
        str(instance),
        "--sysid",
        str(sysid),
        "--use-dir",
        use_dir,
        "--no-rebuild",
    ]

    if not enable_mavproxy:
        cmd.append("--no-mavproxy")
    else:
        if mavproxy_console:
            cmd.append("--console")
        if mavproxy_map:
            cmd.append("--map")

    if custom_location:
        cmd.extend(["--custom-location", custom_location])

    if add_param_file:
        cmd.extend(["--add-param-file", os.path.expanduser(str(add_param_file).strip())])

    rospy.loginfo("Starting SITL session: %s", " ".join(cmd))

    terminal_state = capture_terminal_state()
    env = os.environ.copy()
    configure_ritw_environment(env, enable_mavproxy and mavproxy_console)
    process = subprocess.Popen(cmd, cwd=ardupilot_root, env=env, preexec_fn=os.setsid)
    rospy.on_shutdown(lambda: terminate_process_tree(process))

    try:
        while not rospy.is_shutdown():
            exit_code = process.poll()
            if exit_code is not None:
                return exit_code
            time.sleep(0.2)

        terminate_process_tree(process)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            exit_code = process.poll()
            if exit_code is not None:
                return 0
            time.sleep(0.1)

        rospy.logwarn("SITL session did not exit after SIGTERM, sending SIGKILL")
        terminate_process_tree(process, sig=signal.SIGKILL)
        process.wait()
        return 0
    finally:
        cleanup_console_helpers(instance)
        restore_terminal_state(terminal_state)


if __name__ == "__main__":
    sys.exit(main())
