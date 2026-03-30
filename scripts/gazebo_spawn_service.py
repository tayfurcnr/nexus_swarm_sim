#!/usr/bin/env python3

import os
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from nexus_swarm_sim.srv import SpawnDrone, SpawnDroneResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from model_renderer import render_model
from placement_resolver import resolve_positions
from vehicle_naming import build_model_name, build_ros_namespace


def _vehicle_index(vehicle_name):
    text = str(vehicle_name).strip()
    if text == "main":
        return 1
    return int(text)


class GazeboSpawnService:
    def __init__(self):
        self._drone_prefix = rospy.get_param("~drone_prefix", "nexus")
        self._template_path = os.path.expanduser(
            rospy.get_param("~template_path", "~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf")
        )
        self._enable_gimbal = bool(rospy.get_param("~enable_gimbal", True))
        self._num_drones = int(rospy.get_param("~num_drones", 1))
        self._formation_mode = rospy.get_param("~formation_mode", "fixed")
        self._formation = rospy.get_param("~formation", "line")
        self._formation_set = rospy.get_param("~formation_set", ["line", "triangle", "grid", "circle", "v"])
        self._formation_seed = rospy.get_param("~formation_seed", "auto")
        self._spacing = float(rospy.get_param("~spacing", 2.0))
        self._origin = {
            "x": float(rospy.get_param("~origin/x", 0.0)),
            "y": float(rospy.get_param("~origin/y", 0.0)),
            "z": float(rospy.get_param("~origin/z", 0.2)),
        }

        self._selected_formation, self._positions = resolve_positions(
            self._num_drones,
            self._formation_mode,
            self._formation,
            self._formation_set,
            self._formation_seed,
            self._spacing,
            self._origin,
        )
        rospy.loginfo(
            "[nexus_swarm_sim][spawn_service] ready prefix=%s num_drones=%d formation=%s",
            self._drone_prefix,
            self._num_drones,
            self._selected_formation,
        )

    def handle(self, req):
        try:
            vehicle_name = str(req.vehicle_name).strip()
            if not vehicle_name:
                raise ValueError("vehicle_name is empty")

            drone_index = _vehicle_index(vehicle_name)
            if drone_index < 1 or drone_index > self._num_drones:
                raise ValueError(
                    f"vehicle_name={vehicle_name} resolved to index={drone_index}, outside configured num_drones={self._num_drones}"
                )

            model_name = build_model_name(self._drone_prefix, drone_index)
            robot_namespace = build_ros_namespace(self._drone_prefix, drone_index)
            resolved_x, resolved_y, resolved_z = self._positions[drone_index - 1]
            instance = drone_index - 1
            to_ardupilot_port = 9003 + instance * 10
            from_ardupilot_port = 9002 + instance * 10

            if not os.path.isfile(self._template_path):
                raise FileNotFoundError(f"template not found: {self._template_path}")

            model_xml, gimbal_model_root = render_model(
                self._template_path,
                model_name,
                robot_namespace,
                to_ardupilot_port,
                from_ardupilot_port,
                enable_gimbal=self._enable_gimbal,
            )

            if gimbal_model_root:
                current_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
                os.environ["GAZEBO_MODEL_PATH"] = (
                    f"{gimbal_model_root}:{current_model_path}" if current_model_path else gimbal_model_root
                )

            pose = Pose()
            pose.position.x = resolved_x
            pose.position.y = resolved_y
            pose.position.z = resolved_z
            pose.orientation.w = 1.0

            service_name = "/gazebo/spawn_sdf_model"
            rospy.loginfo(
                "[nexus_swarm_sim][spawn_service] spawning vehicle=%s model=%s ns=%s at (%.2f, %.2f, %.2f)",
                vehicle_name,
                model_name,
                robot_namespace,
                resolved_x,
                resolved_y,
                resolved_z,
            )
            rospy.wait_for_service(service_name, timeout=30.0)
            spawn_model = rospy.ServiceProxy(service_name, SpawnModel)
            resp = spawn_model(model_name, model_xml, robot_namespace, pose, "world")
            if not resp.success:
                raise RuntimeError(resp.status_message)

            return SpawnDroneResponse(
                success=True,
                status_message=f"spawned via {service_name}",
                model_name=model_name,
                robot_namespace=robot_namespace,
                resolved_x=resolved_x,
                resolved_y=resolved_y,
                resolved_z=resolved_z,
            )
        except Exception as exc:
            rospy.logerr("[nexus_swarm_sim][spawn_service] %s", exc)
            return SpawnDroneResponse(
                success=False,
                status_message=str(exc),
                model_name="",
                robot_namespace="",
                resolved_x=0.0,
                resolved_y=0.0,
                resolved_z=0.0,
            )


def main():
    rospy.init_node("gazebo_spawn_service", anonymous=False)
    server = GazeboSpawnService()
    rospy.Service("spawn_drone", SpawnDrone, server.handle)
    rospy.spin()


if __name__ == "__main__":
    main()

