#!/usr/bin/env python3
import math
import os
import random
import signal
import subprocess
import sys
import time

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from vehicle_naming import build_model_name, build_public_id, build_ros_namespace


def spawn_gazebo_model(model_name, drone_namespace, model_file, x, y, z):
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
    rospy.loginfo("Waiting for %s to spawn %s", service_name, model_name)
    rospy.wait_for_service(service_name, timeout=30.0)
    spawn_model = rospy.ServiceProxy(service_name, SpawnModel)

    response = spawn_model(model_name, model_xml, drone_namespace, pose, "world")
    if not response.success:
        raise RuntimeError(f"Failed to spawn {model_name}: {response.status_message}")

    rospy.loginfo("Spawned %s successfully via %s", model_name, service_name)


def _normalize_formation_name(name):
    return str(name).strip().lower()


def _resolve_seed(raw_seed):
    if isinstance(raw_seed, str):
        raw_seed = raw_seed.strip()
        if not raw_seed or raw_seed.lower() == "auto":
            return None
        try:
            return int(raw_seed)
        except ValueError as exc:
            raise ValueError(f"formation_seed must be 'auto' or an integer, got: {raw_seed}") from exc
    if raw_seed is None:
        return None
    return int(raw_seed)


def _resolve_formation(mode, formation, formation_set, seed):
    formation = _normalize_formation_name(formation)
    formation_set = [_normalize_formation_name(item) for item in formation_set if str(item).strip()]
    valid_formations = {"line", "triangle", "grid", "circle", "v", "diamond", "arrowhead", "crescent"}

    if formation not in valid_formations:
        raise ValueError(f"Unsupported formation: {formation}")

    invalid_items = [item for item in formation_set if item not in valid_formations]
    if invalid_items:
        raise ValueError(f"Unsupported formation(s) in formation_set: {invalid_items}")

    if mode == "fixed":
        return formation
    if mode != "random":
        raise ValueError(f"Unsupported formation_mode: {mode}")
    if not formation_set:
        raise ValueError("formation_set must not be empty when formation_mode is 'random'")

    rng = random.Random(seed)
    return rng.choice(formation_set)


def _line_positions(count, spacing, origin):
    return [
        (origin["x"] + float(i) * spacing, origin["y"], origin["z"])
        for i in range(count)
    ]


def _grid_positions(count, spacing, origin):
    columns = int(math.ceil(math.sqrt(count)))
    rows = int(math.ceil(float(count) / columns))
    x_offset = (columns - 1) * spacing / 2.0
    y_offset = (rows - 1) * spacing / 2.0
    positions = []
    for i in range(count):
        row = i // columns
        col = i % columns
        positions.append(
            (
                origin["x"] + col * spacing - x_offset,
                origin["y"] + row * spacing - y_offset,
                origin["z"],
            )
        )
    return positions


def _circle_positions(count, spacing, origin):
    if count == 1:
        return [(origin["x"], origin["y"], origin["z"])]
    radius = max(spacing, spacing * count / (2.0 * math.pi))
    positions = []
    for i in range(count):
        angle = (2.0 * math.pi * i) / count
        positions.append(
            (
                origin["x"] + radius * math.cos(angle),
                origin["y"] + radius * math.sin(angle),
                origin["z"],
            )
        )
    return positions


def _v_positions(count, spacing, origin):
    positions = [(origin["x"], origin["y"], origin["z"])]
    for i in range(1, count):
        arm_index = (i + 1) // 2
        direction = 1.0 if i % 2 else -1.0
        positions.append(
            (
                origin["x"] + arm_index * spacing,
                origin["y"] + direction * arm_index * spacing,
                origin["z"],
            )
        )
    return positions


def _arrowhead_positions(count, spacing, origin):
    positions = [(origin["x"], origin["y"], origin["z"])]
    for i in range(1, count):
        arm_index = (i + 1) // 2
        direction = 1.0 if i % 2 else -1.0
        positions.append(
            (
                origin["x"] - arm_index * spacing,
                origin["y"] + direction * arm_index * spacing,
                origin["z"],
            )
        )
    return positions


def _triangle_positions(count, spacing, origin):
    positions = []
    row = 0
    while len(positions) < count:
        row_count = row + 1
        x = origin["x"] + row * spacing
        y_start = origin["y"] - (row_count - 1) * spacing / 2.0
        for idx in range(row_count):
            positions.append((x, y_start + idx * spacing, origin["z"]))
            if len(positions) == count:
                return positions
        row += 1
    return positions


def _diamond_positions(count, spacing, origin):
    positions = [(origin["x"], origin["y"], origin["z"])]
    ring = 1
    while len(positions) < count:
        for dx in range(-ring, ring + 1):
            dy = ring - abs(dx)
            candidates = [(dx, dy)]
            if dy != 0:
                candidates.append((dx, -dy))
            for cx, cy in candidates:
                positions.append(
                    (
                        origin["x"] + cx * spacing,
                        origin["y"] + cy * spacing,
                        origin["z"],
                    )
                )
                if len(positions) == count:
                    return positions
        ring += 1
    return positions


def _crescent_positions(count, spacing, origin):
    if count == 1:
        return [(origin["x"], origin["y"], origin["z"])]

    radius = max(spacing, spacing * count / math.pi)
    angle_start = -math.pi / 3.0
    angle_end = math.pi / 3.0
    positions = []
    for i in range(count):
        t = i / (count - 1) if count > 1 else 0.0
        angle = angle_start + (angle_end - angle_start) * t
        positions.append(
            (
                origin["x"] - radius * math.cos(angle),
                origin["y"] + radius * math.sin(angle),
                origin["z"],
            )
        )
    return positions


def generate_positions(count, formation, spacing, origin):
    generators = {
        "line": _line_positions,
        "triangle": _triangle_positions,
        "grid": _grid_positions,
        "circle": _circle_positions,
        "v": _v_positions,
        "diamond": _diamond_positions,
        "arrowhead": _arrowhead_positions,
        "crescent": _crescent_positions,
    }
    return generators[formation](count, spacing, origin)


def main():
    rospy.init_node('swarm_launcher', anonymous=True)

    num_drones = rospy.get_param('~num_drones', 3)
    vehicle_model = rospy.get_param('~vehicle_model', rospy.get_param('/vehicle_model', 'iris'))
    drone_prefix = rospy.get_param('~drone_prefix', rospy.get_param('/drone_prefix', 'nexus'))
    spawn_mode = rospy.get_param('~spawn_mode', 'ardupilot')
    spawn_delay = rospy.get_param('~spawn_delay', 2.0)
    gazebo_model_file = rospy.get_param('~gazebo_model_file', '')
    enable_gimbal = rospy.get_param('~enable_gimbal', True)
    enable_mavproxy = rospy.get_param('~enable_mavproxy', True)
    mavproxy_console = rospy.get_param('~mavproxy_console', False)
    mavproxy_map = rospy.get_param('~mavproxy_map', False)
    quiet_mavproxy = rospy.get_param('~quiet_mavproxy', True)
    formation_mode = _normalize_formation_name(rospy.get_param('~formation_mode', 'fixed'))
    formation = rospy.get_param('~formation', 'line')
    formation_set = rospy.get_param('~formation_set', ['line', 'triangle', 'grid', 'circle', 'v'])
    formation_seed = _resolve_seed(rospy.get_param('~formation_seed', 'auto'))
    spacing = float(rospy.get_param('~spacing', 2.0))
    origin = {
        "x": float(rospy.get_param('~origin/x', 0.0)),
        "y": float(rospy.get_param('~origin/y', 0.0)),
        "z": float(rospy.get_param('~origin/z', 0.2)),
    }

    processes = []
    selected_formation = _resolve_formation(formation_mode, formation, formation_set, formation_seed)
    positions = generate_positions(num_drones, selected_formation, spacing, origin)
    rospy.set_param('~selected_formation', selected_formation)

    rospy.loginfo(
        "Starting Swarm Launcher for %d drones (mode=%s, model=%s, prefix=%s, formation=%s, spacing=%.2f)",
        num_drones,
        spawn_mode,
        vehicle_model,
        drone_prefix,
        selected_formation,
        spacing,
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
        drone_index = i + 1
        drone_model_name = build_model_name(drone_prefix, drone_index)
        drone_id = build_public_id(drone_prefix, drone_index)
        drone_ns = build_ros_namespace(drone_prefix, drone_index)
        instance = i
        sys_id = drone_index
        x, y, z = positions[i]

        rospy.loginfo(
            "Spawning %s (model=%s namespace=%s instance=%d sysid=%d)",
            drone_id,
            drone_model_name,
            drone_ns,
            instance,
            sys_id,
        )

        if spawn_mode == "uwb_only":
            cmd = [
                "roslaunch",
                "nexus_swarm_sim", "spawn_dummy.launch",
                f"drone_prefix:={drone_prefix}",
                f"drone_index:={drone_index}",
                f"drone_id:={drone_id}",
                f"drone_model_name:={drone_model_name}",
                f"drone_ns:={drone_ns}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
            ]
        elif spawn_mode == "gazebo_model":
            try:
                spawn_gazebo_model(drone_model_name, drone_ns, gazebo_model_file, x, y, z)
            except Exception as exc:
                rospy.logfatal("Failed to spawn %s: %s", drone_model_name, exc)
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
                f"drone_index:={drone_index}",
                f"drone_id:={drone_id}",
                f"drone_model_name:={drone_model_name}",
                f"drone_ns:={drone_ns}",
                f"instance:={instance}",
                f"sys_id:={sys_id}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
                f"enable_gimbal:={'true' if enable_gimbal else 'false'}",
                f"enable_mavproxy:={'true' if enable_mavproxy else 'false'}",
                f"mavproxy_console:={'true' if mavproxy_console else 'false'}",
                f"mavproxy_map:={'true' if mavproxy_map else 'false'}",
                f"quiet_mavproxy:={'true' if quiet_mavproxy else 'false'}",
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
