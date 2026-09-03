#!/usr/bin/env python3
"""Generate a deployment policy profile from one saved mjlab training run."""

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET


EXPECTED_OBSERVATIONS = (
    "base_ang_vel,projected_gravity,command,phase,joint_pos,joint_vel,actions"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def floats(text: str) -> list[float]:
    return [float(value) for value in text.split(",") if value]


def close_list(left, right, tolerance=1e-6) -> bool:
    return len(left) == len(right) and all(
        abs(a - b) <= tolerance for a, b in zip(left, right)
    )


def canonical_joint_name(name: str) -> str:
    return name.removesuffix("_joint")


def load_training_contract(env_path: Path, yaml_module):
    # BaseLoader preserves the data tree without importing mjlab's Python objects.
    env = yaml_module.load(env_path.read_text(), Loader=yaml_module.BaseLoader)
    robot = env["scene"]["entities"]["robot"]
    initial_positions = {
        canonical_joint_name(name): float(value)
        for name, value in robot["init_state"]["joint_pos"].items()
        if name != ".*"
    }

    actuators = {}
    for actuator in robot["articulation"]["actuators"]:
        targets = actuator["target_names_expr"]
        # DelayedActuatorCfg stores physical PD parameters in base_cfg.
        physical_actuator = actuator.get("base_cfg", actuator)
        contract = {
            "kp": float(physical_actuator["stiffness"]),
            "kd": float(physical_actuator["damping"]),
            "effort_limit": float(physical_actuator["effort_limit"]),
            "velocity_limit": float(physical_actuator["velocity_limit"]),
        }
        for target in targets:
            if ".*" in target:
                raise ValueError(f"actuator target must be explicit: {targets}")
            actuators[canonical_joint_name(target)] = contract.copy()

    action = env["actions"]["joint_pos"]
    if action.get("use_default_offset", "false").lower() != "true":
        raise ValueError("only use_default_offset=true policies are supported")
    action_scales = {
        canonical_joint_name(name): float(value)
        for name, value in action["scale"].items()
    }
    phase_periods = []
    for observation_group in env["observations"].values():
        terms = observation_group.get("terms", observation_group)
        phase = terms.get("phase")
        if phase:
            phase_periods.append(float(phase["params"]["period"]))
    if not phase_periods or any(abs(value - phase_periods[0]) > 1e-9 for value in phase_periods):
        raise ValueError(f"actor/critic gait periods are missing or inconsistent: {phase_periods}")

    actor = env["observations"]["actor"]
    observation_names = list(actor["terms"])
    expected_names = EXPECTED_OBSERVATIONS.split(",")
    if observation_names != expected_names:
        raise ValueError(
            f"actor observation terms are {observation_names}, expected {expected_names}"
        )
    if actor.get("concatenate_terms", "false").lower() != "true":
        raise ValueError("actor observations must concatenate terms")
    if actor.get("flatten_history_dim", "false").lower() != "true":
        raise ValueError("actor observation history must be flattened")
    history_length = int(actor["history_length"])
    if history_length < 1:
        raise ValueError("actor observation history_length must be positive")

    return (
        initial_positions,
        actuators,
        action_scales,
        phase_periods[0],
        history_length,
    )


def load_xml_contract(xml_path: Path):
    root = ET.parse(xml_path).getroot()
    model_name = root.attrib.get("model", "")
    joints = {}
    for joint in root.findall(".//joint"):
        name = joint.attrib.get("name")
        if not name:
            continue
        canonical = canonical_joint_name(name)
        range_text = joint.attrib.get("range")
        if range_text is None:
            raise ValueError(f"joint {name} has no range in robot XML")
        lower, upper = (float(value) for value in range_text.split())
        joints[canonical] = {
            "lower": lower,
            "upper": upper,
            "axis": [float(value) for value in joint.attrib.get("axis", "0 0 1").split()],
        }
    return model_name, joints


def last_tensor_dim(value) -> int:
    return value.type.tensor_type.shape.dim[-1].dim_value


def relative_path(target: Path, parent_file: Path) -> str:
    return os.path.relpath(target, parent_file.parent)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, help="Training run containing params/env.yaml")
    parser.add_argument("--model", required=True, help="Exported ONNX to deploy")
    parser.add_argument("--robot-xml", required=True, help="Authoritative training MJCF XML")
    parser.add_argument("--output", required=True, help="Generated policy profile YAML")
    args = parser.parse_args()

    try:
        import onnx
        import yaml
    except ImportError as exc:
        print(f"FAIL: missing Python dependency: {exc}")
        return 2

    run_dir = Path(args.run_dir).expanduser().resolve()
    env_path = run_dir / "params" / "env.yaml"
    model_path = Path(args.model).expanduser().resolve()
    xml_path = Path(args.robot_xml).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    for path in (env_path, model_path, xml_path):
        if not path.is_file():
            print(f"FAIL: required file does not exist: {path}")
            return 2

    try:
        (
            initial_positions,
            actuators,
            action_scales,
            gait_period,
            history_length,
        ) = load_training_contract(env_path, yaml)
        model_name, xml_joints = load_xml_contract(xml_path)
        if model_name not in {"lowbodycapsulev3", "A00机器人全身", "jljwholebody"}:
            raise ValueError(
                "robot XML model is " + repr(model_name) +
                ", expected a supported JLJ deployment model"
            )

        model = onnx.load(str(model_path))
        metadata = {item.key: item.value for item in model.metadata_props}
        joint_names = [
            canonical_joint_name(name)
            for name in metadata.get("joint_names", "").split(",")
            if name
        ]
        default_positions = floats(metadata.get("default_joint_pos", ""))
        onnx_action_scales = floats(metadata.get("action_scale", ""))
        if not joint_names:
            raise ValueError("ONNX metadata has no joint_names")
        if last_tensor_dim(model.graph.output[0]) != len(joint_names):
            raise ValueError("ONNX action dimension does not match joint_names")
        if metadata.get("observation_names") != EXPECTED_OBSERVATIONS:
            raise ValueError(
                "ONNX observation term order is not the supported deployment contract"
            )
        single_frame_dim = 11 + 3 * len(joint_names)
        expected_input_dim = single_frame_dim * history_length
        if last_tensor_dim(model.graph.input[0]) != expected_input_dim:
            raise ValueError(
                "ONNX input dimension does not match the term-major history contract: "
                f"model={last_tensor_dim(model.graph.input[0])}, expected={expected_input_dim}"
            )

        missing = [
            name
            for name in joint_names
            if name not in actuators or name not in action_scales or name not in xml_joints
        ]
        if missing:
            raise ValueError(f"joints missing from env or XML contract: {missing}")
        env_defaults = [initial_positions.get(name, 0.0) for name in joint_names]
        env_scales = [action_scales[name] for name in joint_names]
        if not close_list(default_positions, env_defaults):
            raise ValueError(
                f"ONNX default positions differ from saved env: {default_positions} vs {env_defaults}"
            )
        # The current exporter formats this metadata to three decimal places. The
        # scale is applied outside the network, so the saved env remains authoritative.
        if not close_list(onnx_action_scales, env_scales, tolerance=5e-4):
            raise ValueError(
                f"ONNX action scales differ from saved env: {onnx_action_scales} vs {env_scales}"
            )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        xml_snapshot = output_path.with_suffix(".mjcf.xml")
        onnx_snapshot = output_path.with_suffix(".onnx")
        shutil.copy2(xml_path, xml_snapshot)
        if model_path != onnx_snapshot:
            shutil.copy2(model_path, onnx_snapshot)
        profile = {
            "policy_profile": {
                "schema_version": 2,
                "source": {
                    "training_run": str(run_dir),
                    "env_yaml": str(env_path),
                    "env_yaml_sha256": sha256(env_path),
                    "original_onnx": str(model_path),
                    "onnx_sha256": sha256(onnx_snapshot),
                    "original_robot_xml": str(xml_path),
                },
                "robot_model": {
                    "name": model_name,
                    "xml_snapshot": relative_path(xml_snapshot, output_path),
                    "xml_sha256": sha256(xml_snapshot),
                },
                "model_path": relative_path(onnx_snapshot, output_path),
                "expected_observation_dim": last_tensor_dim(model.graph.input[0]),
                "expected_action_dim": last_tensor_dim(model.graph.output[0]),
                "observation_layout": "term_major",
                "observation_history_length": history_length,
                "observation_terms": [
                    {"name": "base_ang_vel", "size": 3},
                    {"name": "projected_gravity", "size": 3},
                    {"name": "command", "size": 3},
                    {"name": "phase", "size": 2},
                    {"name": "joint_pos", "size": len(joint_names)},
                    {"name": "joint_vel", "size": len(joint_names)},
                    {"name": "actions", "size": len(joint_names)},
                ],
                "gait_period_s": gait_period,
                "joint_names": joint_names,
                "default_joint_positions": env_defaults,
                "action_scale": {
                    name: action_scales[name] for name in joint_names
                },
                "joint_limits": {
                    name: [xml_joints[name]["lower"], xml_joints[name]["upper"]]
                    for name in joint_names
                },
                "joint_axes": {
                    name: xml_joints[name]["axis"] for name in joint_names
                },
                "actuators": {name: actuators[name] for name in joint_names},
            }
        }
        output_path.write_text(
            "# Generated by generate_policy_profile.py; do not edit by hand.\n"
            + yaml.safe_dump(profile, sort_keys=False),
            encoding="utf-8",
        )
    except (KeyError, TypeError, ValueError, ET.ParseError) as exc:
        print(f"FAIL: {exc}")
        return 1

    print(f"PASS: generated {output_path}")
    print(f"      ONNX snapshot {onnx_snapshot}")
    print(f"      XML snapshot {xml_snapshot}")
    print(f"      run={run_dir.name}, model={model_name}, joints={len(joint_names)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
