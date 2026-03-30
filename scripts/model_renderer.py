#!/usr/bin/env python3

import os
import shutil


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

