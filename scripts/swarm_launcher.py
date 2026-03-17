#!/usr/bin/env python3
import rospy
import subprocess
import time
import os
import signal
import sys


def main():
    rospy.init_node('swarm_launcher', anonymous=True)

    num_drones = rospy.get_param('~num_drones', 3)
    vehicle_model = rospy.get_param('~vehicle_model', rospy.get_param('/vehicle_model', 'iris'))
    drone_prefix = rospy.get_param('~drone_prefix', rospy.get_param('/drone_prefix', 'nexus'))
    spawn_mode = rospy.get_param('~spawn_mode', 'ardupilot')
    spawn_delay = rospy.get_param('~spawn_delay', 2.0)
    gazebo_model_file = rospy.get_param('~gazebo_model_file', '')

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
        drone_id = f"{drone_prefix}{i}"
        instance = i
        sys_id = i + 1
        x = float(i) * 2.0
        y = 0.0
        z = 0.2

        rospy.loginfo(f"Spawning {drone_id} (Instance: {instance}, SysID: {sys_id})")

        if spawn_mode == "uwb_only":
            cmd = [
                "roslaunch",
                "swarm_gazebo_sim", "spawn_dummy.launch",
                f"drone_prefix:={drone_prefix}",
                f"drone_id:={drone_id}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
            ]
        elif spawn_mode == "gazebo_model":
            cmd = [
                "roslaunch",
                "swarm_gazebo_sim", "spawn_model.launch",
                f"drone_prefix:={drone_prefix}",
                f"drone_id:={drone_id}",
                f"model_file:={gazebo_model_file}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
            ]
        else:
            cmd = [
                "roslaunch",
                "swarm_gazebo_sim", "spawn_sitl.launch",
                f"vehicle_model:={vehicle_model}",
                f"drone_prefix:={drone_prefix}",
                f"drone_id:={drone_id}",
                f"instance:={instance}",
                f"sys_id:={sys_id}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
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
