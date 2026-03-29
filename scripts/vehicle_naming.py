#!/usr/bin/env python3
"""Helpers for mapping flat Gazebo model names to hierarchical ROS IDs."""

from __future__ import annotations


def _parse_index(value):
    try:
        return int(str(value))
    except (TypeError, ValueError):
        return None


def build_model_name(prefix, index):
    return f"{prefix}{int(index)}"


def build_public_id(prefix, index):
    return f"{prefix}/{int(index)}"


def build_ros_namespace(prefix, index):
    return f"/{build_public_id(prefix, index)}"


def parse_model_name(model_name, prefix):
    if not model_name.startswith(prefix):
        return None
    suffix = model_name[len(prefix):]
    if not suffix or not suffix.isdigit():
        return None
    return int(suffix)


def model_name_to_public_id(model_name, prefix):
    index = parse_model_name(model_name, prefix)
    if index is None:
        return model_name
    return build_public_id(prefix, index)


def model_name_to_ros_namespace(model_name, prefix):
    index = parse_model_name(model_name, prefix)
    if index is None:
        return f"/{model_name}"
    return build_ros_namespace(prefix, index)


def normalize_public_id(vehicle_id, prefix=None):
    vehicle_id = str(vehicle_id).strip().strip("/")
    if not vehicle_id:
        return vehicle_id

    if "/" in vehicle_id:
        parts = [part for part in vehicle_id.split("/") if part]
        if len(parts) == 2:
            index = _parse_index(parts[1])
            if index is not None:
                return build_public_id(parts[0], index)
        return "/".join(parts)

    if prefix:
        index = parse_model_name(vehicle_id, prefix)
        if index is not None:
            return build_public_id(prefix, index)

    return vehicle_id


def public_id_to_ros_namespace(vehicle_id):
    normalized = normalize_public_id(vehicle_id)
    return f"/{normalized}" if normalized else "/"


def public_id_to_model_name(vehicle_id, prefix=None):
    normalized = normalize_public_id(vehicle_id, prefix=prefix)
    parts = [part for part in normalized.split("/") if part]
    if len(parts) == 2 and parts[1].isdigit():
        return build_model_name(parts[0], int(parts[1]))
    return normalized.replace("/", "")


def vehicle_sort_key(vehicle_id, prefix=None):
    normalized = normalize_public_id(vehicle_id, prefix=prefix)
    parts = [part for part in normalized.split("/") if part]
    if len(parts) == 2 and parts[1].isdigit():
        return (parts[0], int(parts[1]))
    return (normalized, 0)
