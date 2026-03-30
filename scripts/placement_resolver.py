#!/usr/bin/env python3

import math
import random


def normalize_formation_name(name):
    return str(name).strip().lower()


def resolve_seed(raw_seed):
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


def resolve_formation(mode, formation, formation_set, seed):
    formation = normalize_formation_name(formation)
    formation_set = [normalize_formation_name(item) for item in formation_set if str(item).strip()]
    valid_formations = set(POSITION_GENERATORS.keys())

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
    return POSITION_GENERATORS[formation](count, spacing, origin)


POSITION_GENERATORS = {
    "line": _line_positions,
    "triangle": _triangle_positions,
    "grid": _grid_positions,
    "circle": _circle_positions,
    "v": _v_positions,
    "diamond": _diamond_positions,
    "arrowhead": _arrowhead_positions,
    "crescent": _crescent_positions,
}


def resolve_positions(count, formation_mode, formation, formation_set, formation_seed, spacing, origin):
    seed = resolve_seed(formation_seed)
    selected = resolve_formation(
        normalize_formation_name(formation_mode),
        formation,
        formation_set,
        seed,
    )
    return selected, generate_positions(count, selected, spacing, origin)
