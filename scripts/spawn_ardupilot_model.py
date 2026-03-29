#!/usr/bin/env python3

import os
import shutil
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def _gimbal_model_source_path():
    candidates = [
        os.path.expanduser("~/.gazebo/models/gimbal_small_2d"),
        os.path.expanduser("~/Downloads/models/gimbal_small_2d"),
    ]
    for path in candidates:
        if os.path.isfile(os.path.join(path, "model.sdf")):
            return path
    raise FileNotFoundError("gimbal_small_2d model directory not found in known Gazebo model paths")


def _prepare_namespaced_gimbal_model(model_name, robot_namespace):
    source_dir = _gimbal_model_source_path()
    target_root = f"/tmp/nexus_swarm_sim_models/{model_name}"
    target_dir = os.path.join(target_root, "gimbal_small_2d")
    os.makedirs(target_root, exist_ok=True)
    if os.path.isdir(target_dir):
        shutil.rmtree(target_dir)
    shutil.copytree(source_dir, target_dir)

    model_sdf_path = os.path.join(target_dir, "model.sdf")
    with open(model_sdf_path, "r", encoding="utf-8") as f:
        gimbal_model = f.read()

    gimbal_model = gimbal_model.replace(
        "<plugin name=\"camera_controller\" filename=\"libgazebo_ros_camera.so\">",
        "<plugin name=\"camera_controller\" filename=\"libgazebo_ros_camera.so\">\n"
        f"          <robotNamespace>{robot_namespace}</robotNamespace>",
        1,
    )
    gimbal_model = gimbal_model.replace(
        "<frameName>gimbal_camera_optical_frame</frameName>",
        f"<frameName>{model_name}/gimbal_camera_optical_frame</frameName>",
        1,
    )

    with open(model_sdf_path, "w", encoding="utf-8") as f:
        f.write(gimbal_model)

    return target_root, target_dir


def render_model(template_path, model_name, robot_namespace, to_ardupilot_port, from_ardupilot_port, enable_gimbal=True):
    with open(template_path, "r", encoding="utf-8") as f:
        content = f.read()

    # The ArduPilot Gazebo template declares a xacro XML namespace that is not
    # used by the generated SDF. Gazebo's parser can mis-handle that extra
    # namespace on later multi-vehicle spawns, so strip it from the rendered XML.
    content = content.replace(" xmlns:xacro='http://ros.org/wiki/xacro'", "", 1)
    content = content.replace(' xmlns:xacro="http://ros.org/wiki/xacro"', "", 1)

    content = content.replace('<model name="iris_demo">', f'<model name="{model_name}">', 1)
    content = content.replace("iris_demo::iris::iris/imu_link::imu_sensor", f"{model_name}::iris::iris/imu_link::imu_sensor")
    content = content.replace("<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{from_ardupilot_port}</fdm_port_in>", 1)
    content = content.replace("<fdm_port_out>9003</fdm_port_out>", f"<fdm_port_out>{to_ardupilot_port}</fdm_port_out>", 1)

    gimbal_model_root = ""
    if enable_gimbal:
        gimbal_model_root, gimbal_model_dir = _prepare_namespaced_gimbal_model(model_name, robot_namespace)
        content = content.replace(
            "<uri>model://gimbal_small_2d</uri>",
            f"<uri>file://{gimbal_model_dir}</uri>",
            1,
        )
    else:
        content = content.replace(
            """
    <include>
      <uri>model://gimbal_small_2d</uri>
      <pose>0 -0.01 0.070 1.57 0 1.57</pose>
    </include>
""".strip(),
            "",
            1,
        )
        content = content.replace(
            """
    <joint name="iris_gimbal_mount" type="revolute">
      <parent>iris::base_link</parent>
      <child>gimbal_small_2d::base_link</child>
      <axis>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
        </limit>
        <xyz>0 0 1</xyz>
        <use_parent_model_frame>true</use_parent_model_frame>
      </axis>
    </joint>
""".strip(),
            "",
            1,
        )

    return content, gimbal_model_root


def main():
    rospy.init_node("ardupilot_model_spawner", anonymous=False)

    template_path = os.path.expanduser(
        rospy.get_param("~template_path", "~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf")
    )
    model_name = rospy.get_param("~model_name")
    robot_namespace = rospy.get_param("~robot_namespace", f"/{model_name}")
    reference_frame = rospy.get_param("~reference_frame", "world")
    to_ardupilot_port = int(rospy.get_param("~to_ardupilot_port"))
    from_ardupilot_port = int(rospy.get_param("~from_ardupilot_port"))
    output_path = rospy.get_param("~output_path", "")
    enable_gimbal = bool(rospy.get_param("~enable_gimbal", True))

    if not os.path.isfile(template_path):
        rospy.logfatal("ArduPilot model template not found: %s", template_path)
        return 1

    model_xml, gimbal_model_root = render_model(
        template_path,
        model_name,
        robot_namespace,
        to_ardupilot_port,
        from_ardupilot_port,
        enable_gimbal=enable_gimbal,
    )

    if gimbal_model_root:
        current_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
        os.environ["GAZEBO_MODEL_PATH"] = f"{gimbal_model_root}:{current_model_path}" if current_model_path else gimbal_model_root

    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(model_xml)

    pose = Pose()
    pose.position.x = float(rospy.get_param("~x", 0.0))
    pose.position.y = float(rospy.get_param("~y", 0.0))
    pose.position.z = float(rospy.get_param("~z", 0.2))
    pose.orientation.w = 1.0

    service_name = "/gazebo/spawn_sdf_model"
    rospy.loginfo("Waiting for %s", service_name)
    rospy.wait_for_service(service_name, timeout=30.0)
    spawn_model = rospy.ServiceProxy(service_name, SpawnModel)

    rospy.loginfo(
        "Spawning %s with fdm_in=%d fdm_out=%d at (%.2f, %.2f, %.2f)",
        model_name,
        from_ardupilot_port,
        to_ardupilot_port,
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )
    # Provide a per-vehicle ROS namespace so Gazebo ROS plugins inside the model
    # don't all publish into the global /gimbal namespace in multi-vehicle runs.
    resp = spawn_model(model_name, model_xml, robot_namespace, pose, reference_frame)
    if not resp.success:
        rospy.logfatal("Failed to spawn %s: %s", model_name, resp.status_message)
        return 1

    rospy.loginfo("Spawned %s successfully", model_name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
