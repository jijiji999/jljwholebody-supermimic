#!/usr/bin/env python3
"""Pack the JLJ SuperMimic export for native deployment."""

import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def prepare(source: Path, output: Path) -> None:
    assets = source / "assets"
    config = yaml.safe_load((source / "config/supermimic.yaml").read_text())
    policy = config["policy"]
    if (
        policy["history_length"], policy["future_frames"],
        policy["observation_dim"], policy["goal_dim"], policy["action_dim"],
        policy["control_frequency"], policy["trajectory_fps"],
    ) != (10, 10, 930, 680, 29, 50, 50):
        raise ValueError("unexpected SuperMimic export contract")

    # Read the order used by the upstream ROS and MuJoCo reference without importing it.
    core = ast.parse((source / "supermimic_deploy/ros2_core.py").read_text())
    joint_order, = (
        ast.literal_eval(statement.value)
        for statement in core.body
        if isinstance(statement, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "JOINTS"
                for target in statement.targets)
    )
    if len(joint_order) != 29 or len(set(joint_order)) != 29:
        raise ValueError("the reference must declare 29 unique policy joints")

    default_file = assets / "default_joint_pos.npy"
    trajectory_file = source / config["trajectory"]["path"]
    goal_file = trajectory_file.with_name(trajectory_file.stem + "_goals.npz")
    default = np.load(default_file).astype(np.float32)
    if default.shape != (29,) or not np.isfinite(default).all():
        raise ValueError("invalid SuperMimic default pose")
    with np.load(trajectory_file) as motion, np.load(goal_file) as packed:
        names = [str(name).removesuffix("_joint") for name in motion["joint_names"]]
        goals = np.asarray(packed["goal"], dtype="<f4", order="C")
        if names != joint_order or motion["fps"].item() != 50 or packed["fps"].item() != 50:
            raise ValueError("reference joint order or frame rate differs from the policy contract")
        if goals.ndim != 2 or goals.shape[0] == 0 or goals.shape[1] != 680:
            raise ValueError("expected a nonempty [frames, 680] goal array")
        if not np.isfinite(goals).all():
            raise ValueError("goal frames contain NaN or Inf")
        if not (np.array_equal(goals[:, :29], motion["joint_pos"] - default) and
                np.array_equal(goals[:, 29:58], motion["joint_vel"])):
            raise ValueError("goals do not match the supplied reference and default pose")

    limit_xml = source / config["robot"]["mujoco_xml"]
    limits = {
        joint.get("name").removesuffix("_joint"): joint.get("range")
        for joint in ET.parse(limit_xml).iter("joint")
        if joint.get("name") and joint.get("range")
    }
    simulation_xml = assets / "A00_robot_mujoco_supermimic.xml"
    servos = {servo.get("joint"): servo for servo in ET.parse(simulation_xml).iter("position")}
    joints = []
    for index, name in enumerate(joint_order):
        lower, upper = map(float, limits[name].split())
        servo = servos[name]
        negative_effort, effort = map(float, servo.get("forcerange").split())
        kp, kd = float(servo.get("kp")), float(servo.get("kv"))
        if servo.get("forcelimited") != "true" or negative_effort != -effort or effort <= 0:
            raise ValueError(f"unexpected actuator effort range for {name}")
        joints.append(dict(
            name=name, default_position=float(default[index]), lower=lower, upper=upper,
            kp=kp, kd=kd, effort_limit=effort,
        ))

    model = source / policy["model_path"]
    weights = model.with_name(model.name + ".data")
    provenance = {}
    for path in (model, weights, default_file, trajectory_file, goal_file, limit_xml, simulation_xml):
        with path.open("rb") as stream:
            if stream.read(10) == b"version ht":
                raise ValueError(f"{path} is a Git LFS pointer. Fetch the binary with git lfs pull.")
            stream.seek(0)
            provenance[str(path.relative_to(source))] = hashlib.sha256(stream.read()).hexdigest()

    if output.exists():
        raise FileExistsError(f"bundle already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as tmp:
        bundle = Path(tmp) / "bundle"
        bundle.mkdir()
        shutil.copyfile(model, bundle / "policy.onnx")
        shutil.copyfile(weights, bundle / "policy.onnx.data")
        goals.tofile(bundle / "goals.f32")
        profile = dict(
            schema_version=1, fps=50, frame_count=len(goals), joints=joints,
            action_scale=config["robot"]["action_scale"], raw_action_clip=policy["raw_action_clip"],
            source_sha256=provenance,
        )
        (bundle / "profile.yaml").write_text(json.dumps(profile, indent=2, allow_nan=False) + "\n")
        bundle.rename(output)
    print(f"Packed {len(goals)} SuperMimic frames in {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="jlj_robot checkout with Git LFS assets")
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).resolve().parents[1] / "config/policies/supermimic/bundle",
    )
    args = parser.parse_args()
    prepare(args.source.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
