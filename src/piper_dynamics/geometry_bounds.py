#!/usr/bin/env python3
"""Geometry support constraints derived from AgileX PiPER X collision meshes."""
from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from .paths import MESH_ROOT
SOURCE_COMMIT = "680fa925cd00db7e2628563d2edc5540963045de"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_binary_stl(path: Path) -> np.ndarray:
    """Read STL triangle vertices without requiring an extra mesh package."""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is truncated: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL with 50-byte facets: {path}")
    # Each facet starts with a 12-byte normal, then 36 bytes of vertices,
    # and a 2-byte attribute field.
    vertices = np.ndarray(
        (count, 3, 3), dtype="<f4", buffer=data, offset=96,
        strides=(50, 12, 4),
    )
    points = vertices.reshape(-1, 3).astype(float)
    if not np.all(np.isfinite(points)):
        raise ValueError(f"STL has non-finite vertices: {path}")
    return points


def transform(points: np.ndarray, xyz: tuple[float, ...],
              rpy: tuple[float, ...]) -> np.ndarray:
    R = Rotation.from_euler("xyz", rpy).as_matrix()
    return points @ R.T + np.asarray(xyz, dtype=float)


def _link6_aggregate_points() -> tuple[np.ndarray, dict[str, str]]:
    """Union link6, flange and gripper collision meshes over the gripper range."""
    source_files = {
        "link6": MESH_ROOT / "arm" / "link6.stl",
        "flange": MESH_ROOT / "arm" / "flange.stl",
        "gripper_base": MESH_ROOT / "gripper" / "gripper_base.stl",
        "gripper_link1": MESH_ROOT / "gripper" / "gripper_link1.stl",
        "gripper_link2": MESH_ROOT / "gripper" / "gripper_link2.stl",
    }
    points = [read_binary_stl(source_files["link6"]),
              read_binary_stl(source_files["flange"])]

    # Xacro: gripper_base_joint = xyz(0,0,0.0045), rpy(0,0,pi/2).
    base_xyz = np.array([0.0, 0.0, 0.0045])
    base_rpy = np.array([0.0, 0.0, np.pi / 2.0])
    R_base = Rotation.from_euler("xyz", base_rpy).as_matrix()
    points.append(transform(read_binary_stl(source_files["gripper_base"]),
                            base_xyz, base_rpy))

    finger_specs = (
        ("gripper_link1", np.array([np.pi/2, 0.0, 0.0]),
         np.array([0.0, 0.0, 1.0]), (0.0, 0.05)),
        ("gripper_link2", np.array([np.pi/2, 0.0, -np.pi]),
         np.array([0.0, 0.0, -1.0]), (-0.05, 0.0)),
    )
    joint_xyz = np.array([0.0, 0.0, 0.138])
    for name, joint_rpy, axis, positions in finger_specs:
        finger_points = read_binary_stl(source_files[name])
        R_joint = Rotation.from_euler("xyz", joint_rpy).as_matrix()
        for position in positions:
            # Apply the prismatic offset in the joint frame, then the fixed
            # joint transform and gripper-base transform into link6.
            local = finger_points + position * axis
            in_base = local @ R_joint.T + joint_xyz
            in_link6 = in_base @ R_base.T + base_xyz
            points.append(in_link6)

    hashes = {key: sha256(path) for key, path in source_files.items()}
    return np.concatenate(points, axis=0), hashes


def derive_link_geometry(margin_m: float = 0.01) -> tuple[dict[str, dict], dict]:
    """Return convex-hull COM planes and second-moment box limits per link.

    Each convex-hull equation is normalized and has the form n @ x + d <= 0.
    A mass distribution supported inside the mesh hull with an outward margin
    ``margin_m`` therefore satisfies n @ h + (d-margin_m) m <= 0.
    The second-moment diagonal obeys Sigma[j,j] <= m * r[j]**2 for a box
    enclosing the same collision geometry and margin.
    """
    if margin_m < 0:
        raise ValueError("geometry margin must be nonnegative")
    result: dict[str, dict] = {}
    sources: dict[str, str] = {}
    for i in range(1, 6):
        path = MESH_ROOT / "arm" / f"link{i}.stl"
        points = read_binary_stl(path)
        sources[f"link{i}"] = sha256(path)
        hull = ConvexHull(np.unique(points, axis=0))
        result[f"link{i}"] = _serialize_hull(hull, points, margin_m)

    link6_points, link6_hashes = _link6_aggregate_points()
    sources.update(link6_hashes)
    hull6 = ConvexHull(np.unique(link6_points, axis=0))
    result["link6"] = _serialize_hull(hull6, link6_points, margin_m)
    manifest = {
        "source_repo": "https://github.com/agilexrobotics/agx_arm_urdf",
        "source_commit": SOURCE_COMMIT,
        "source_subtrees": ["piper_x/meshes", "agx_gripper/meshes"],
        "mesh_sha256": sources,
        "geometry_model": "collision-mesh convex hull; link6 includes flange and gripper meshes at URDF positions over the stated prismatic limits",
        "link6_gripper_positions_m": {"gripper_joint1": [0.0, 0.05], "gripper_joint2": [-0.05, 0.0]},
        "outward_margin_m": float(margin_m),
        "limitations": [
            "The mesh hull is an outer support bound, not a measured internal mass distribution.",
            "The 10 mm default margin accommodates collision-mesh and hardware differences.",
            "Link6 geometry spans the Xacro finger limits; the fitted inertial model still uses the nominal closed-gripper aggregate.",
        ],
    }
    return result, manifest


def _serialize_hull(hull: ConvexHull, points: np.ndarray,
                    margin_m: float) -> dict:
    low = points.min(axis=0)
    high = points.max(axis=0)
    radius = np.maximum(np.abs(low), np.abs(high)) + margin_m
    equations = hull.equations.astype(float)
    norms = np.linalg.norm(equations[:, :3], axis=1)
    equations /= norms[:, None]
    return {
        "aabb_min_m": low.tolist(),
        "aabb_max_m": high.tolist(),
        "second_moment_radius_m": radius.tolist(),
        "hull_vertex_count": int(len(hull.vertices)),
        "hull_facet_count": int(len(equations)),
        "hull_equations_n_dot_x_plus_d_le_zero": equations.tolist(),
    }


if __name__ == "__main__":
    data, manifest = derive_link_geometry()
    print(json.dumps({
        "manifest": manifest,
        "links": {k: {p: v for p, v in val.items() if p != "hull_equations_n_dot_x_plus_d_le_zero"}
                  for k, val in data.items()},
    }, indent=2))
