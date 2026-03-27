#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def spawn_gazebo_model(drone_id, model_file, x, y, z):
    if not model_file:
        raise ValueError("gazebo_model_file is empty")
    if not os.path.isfile(model_file):
        raise FileNotFoundError(f"Gazebo model file not found: {model_file}")

    with open(model_file, "r", encoding="utf-8") as handle:
        model_xml = handle.read()

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    service_name = "/gazebo/spawn_sdf_model"
    rospy.loginfo("Waiting for %s to spawn %s", service_name, drone_id)
    rospy.wait_for_service(service_name, timeout=30.0)
    spawn_model = rospy.ServiceProxy(service_name, SpawnModel)

    response = spawn_model(drone_id, model_xml, f"/{drone_id}", pose, "world")
    if not response.success:
        raise RuntimeError(f"Failed to spawn {drone_id}: {response.status_message}")

    rospy.loginfo("Spawned %s successfully via %s", drone_id, service_name)


def main():
    rospy.init_node('swarm_launcher', anonymous=True)

    num_drones = rospy.get_param('~num_drones', 3)
    vehicle_model = rospy.get_param('~vehicle_model', rospy.get_param('/vehicle_model', 'iris'))
    drone_prefix = rospy.get_param('~drone_prefix', rospy.get_param('/drone_prefix', 'nexus'))
    spawn_mode = rospy.get_param('~spawn_mode', 'ardupilot')
    spawn_delay = rospy.get_param('~spawn_delay', 2.0)
    gazebo_model_file = rospy.get_param('~gazebo_model_file', '')
    enable_gimbal = rospy.get_param('~enable_gimbal', True)
    enable_mavproxy = rospy.get_param('~enable_mavproxy', False)
    mavproxy_console = rospy.get_param('~mavproxy_console', False)
    mavproxy_map = rospy.get_param('~mavproxy_map', False)

    processes = []

    rospy.loginfo(
        "Starting Swarm Launcher for %d drones (mode=%s, model=%s, prefix=%s)",
        num_drones,
        spawn_mode,
        vehicle_model,
        drone_prefix,
    )

    def terminate_children():
        for proc in processes:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass

    rospy.on_shutdown(terminate_children)

    for i in range(num_drones):
        drone_id = f"{drone_prefix}{i + 1}"
        instance = i
        sys_id = i + 1
        x = float(i) * 2.0
        y = 0.0
        z = 0.2

        rospy.loginfo(f"Spawning {drone_id} (Instance: {instance}, SysID: {sys_id})")

        if spawn_mode == "uwb_only":
            cmd = [
                "roslaunch",
                "nexus_swarm_sim", "spawn_dummy.launch",
                f"drone_prefix:={drone_prefix}",
                f"drone_id:={drone_id}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
            ]
        elif spawn_mode == "gazebo_model":
            try:
                spawn_gazebo_model(drone_id, gazebo_model_file, x, y, z)
            except Exception as exc:
                rospy.logfatal("Failed to spawn %s: %s", drone_id, exc)
                terminate_children()
                sys.exit(1)
            if i + 1 < num_drones:
                time.sleep(spawn_delay)
            continue
        else:
            cmd = [
                "roslaunch",
                "nexus_swarm_sim", "spawn_sitl.launch",
                f"vehicle_model:={vehicle_model}",
                f"drone_prefix:={drone_prefix}",
                f"drone_id:={drone_id}",
                f"instance:={instance}",
                f"sys_id:={sys_id}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
                f"enable_gimbal:={'true' if enable_gimbal else 'false'}",
                f"enable_mavproxy:={'true' if enable_mavproxy else 'false'}",
                f"mavproxy_console:={'true' if mavproxy_console else 'false'}",
                f"mavproxy_map:={'true' if mavproxy_map else 'false'}",
            ]

        proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        processes.append(proc)

        if i + 1 < num_drones:
            time.sleep(spawn_delay)

    rospy.loginfo("All drones spawned. Waiting for shutdown.")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        dead_children = [proc for proc in processes if proc.poll() not in (None, 0)]
        if dead_children:
            rospy.logwarn("One or more drone launch processes exited unexpectedly.")
            break
        rate.sleep()

    terminate_children()
    sys.exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
