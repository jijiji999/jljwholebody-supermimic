#!/usr/bin/env python3
"""Validate ONNX, saved env, generated profile, and robot XML as one contract."""

import argparse
import hashlib
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from generate_policy_profile import (
    EXPECTED_OBSERVATIONS,
    canonical_joint_name,
    close_list,
    floats,
    load_training_contract,
    load_xml_contract,
)


ANKLE_LIMITS = {
    "left_ankle_pitch": [-0.5, 0.3],
    "left_ankle_roll": [-0.2, 0.2],
    "right_ankle_pitch": [-0.5, 0.3],
    "right_ankle_roll": [-0.2, 0.2],
    "waist_pitch": [-0.5, 0.5],
    "waist_roll": [-0.2, 0.2],
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve(parent_file: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (parent_file.parent / path).resolve()


def load_selected_profile(selector_path: Path, yaml_module):
    """Load either a selector active.yaml or a legacy full profile."""
    document = yaml_module.safe_load(selector_path.read_text()) or {}
    if "policy_profile" in document:
        return selector_path, document["policy_profile"]
    selected = document.get("active_profile")
    if document.get("schema_version") != 1 or not isinstance(selected, str) or not selected:
        raise ValueError(
            "active policy selector must contain schema_version: 1 and active_profile"
        )
    profile_path = resolve(selector_path, selected)
    profile_document = yaml_module.safe_load(profile_path.read_text()) or {}
    if "policy_profile" not in profile_document:
        raise ValueError(f"selected profile has no policy_profile root: {profile_path}")
    return profile_path, profile_document["policy_profile"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--model", default="", help="Override the profile model_path")
    parser.add_argument("--profile", default="", help="Validate this profile instead of the active one")
    parser.add_argument(
        "--urdf-xacro",
        default="",
        help="Optionally verify revolute joint axes/limits against this Xacro",
    )
    args = parser.parse_args()

    try:
        import onnx
        import yaml
    except ImportError as exc:
        print(f"FAIL: missing Python dependency: {exc}")
        return 2

    config_path = Path(args.config).expanduser().resolve()
    deploy = yaml.safe_load(config_path.read_text())["deployment"]
    policy = deploy["policy"]
    profile_value = args.profile or policy.get("profile_path", "")
    if not profile_value:
        print("FAIL: a policy profile is required for a traceable policy")
        return 1
    selector_path = (
        Path(profile_value).expanduser().resolve()
        if args.profile
        else resolve(config_path, profile_value)
    )
    try:
        profile_path, profile = load_selected_profile(selector_path, yaml)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"FAIL: cannot resolve active policy selector: {exc}")
        return 1

    model_path = (
        Path(args.model).expanduser().resolve()
        if args.model
        else resolve(profile_path, profile["model_path"])
    )
    env_path = resolve(profile_path, profile["source"]["env_yaml"])
    xml_snapshot = resolve(profile_path, profile["robot_model"]["xml_snapshot"])
    model = onnx.load(str(model_path))
    metadata = {item.key: item.value for item in model.metadata_props}

    def last_dim(value):
        return value.type.tensor_type.shape.dim[-1].dim_value

    results = []

    def check(name, passed, detail=""):
        results.append(bool(passed))
        suffix = f" ({detail})" if detail else ""
        print(f"[{'PASS' if passed else 'FAIL'}] {name}{suffix}")

    print(f"Deploy:  {config_path}")
    if profile_path != selector_path:
        print(f"Active:  {selector_path}")
    print(f"Profile: {profile_path}")
    print(f"Policy:  {model_path}")
    print(f"Env:     {env_path}")
    print(f"MJCF:    {xml_snapshot}")

    try:
        check("profile schema", profile.get("schema_version") == 2)
        check(
            "ONNX hash",
            sha256(model_path) == profile["source"]["onnx_sha256"],
        )
        check(
            "saved env hash",
            sha256(env_path) == profile["source"]["env_yaml_sha256"],
        )
        check(
            "robot XML snapshot hash",
            sha256(xml_snapshot) == profile["robot_model"]["xml_sha256"],
        )

        joints = profile["joint_names"]
        model_joints = [
            canonical_joint_name(name)
            for name in metadata.get("joint_names", "").split(",")
            if name
        ]
        scales = [profile["action_scale"][name] for name in joints]
        check("observation dimension", last_dim(model.graph.input[0]) == profile["expected_observation_dim"])
        check("action dimension", last_dim(model.graph.output[0]) == profile["expected_action_dim"])
        check("joint order", model_joints == joints)
        check(
            "ONNX default positions",
            close_list(
                floats(metadata.get("default_joint_pos", "")),
                profile["default_joint_positions"],
                tolerance=1e-6,
            ),
        )
        check(
            "ONNX action scale metadata",
            close_list(
                floats(metadata.get("action_scale", "")), scales, tolerance=5e-4 + 1e-12
            ),
            "exporter metadata is rounded; saved env values are used at runtime",
        )
        check(
            "observation term order",
            metadata.get("observation_names") == EXPECTED_OBSERVATIONS,
        )

        (
            env_defaults,
            env_actuators,
            env_scales,
            gait_period,
            history_length,
        ) = load_training_contract(env_path, yaml)
        expected_terms = [
            {"name": "base_ang_vel", "size": 3},
            {"name": "projected_gravity", "size": 3},
            {"name": "command", "size": 3},
            {"name": "phase", "size": 2},
            {"name": "joint_pos", "size": len(joints)},
            {"name": "joint_vel", "size": len(joints)},
            {"name": "actions", "size": len(joints)},
        ]
        check("observation layout", profile.get("observation_layout") == "term_major")
        check(
            "observation history length",
            profile.get("observation_history_length") == history_length,
        )
        check("observation term sizes", profile.get("observation_terms") == expected_terms)
        check(
            "history observation dimension",
            profile["expected_observation_dim"]
            == history_length * sum(term["size"] for term in expected_terms),
        )
        check(
            "saved env default positions",
            close_list(
                [env_defaults.get(name, 0.0) for name in joints],
                profile["default_joint_positions"],
                tolerance=1e-9,
            ),
        )
        check(
            "saved env action scales",
            close_list([env_scales[name] for name in joints], scales, tolerance=1e-12),
        )
        check("saved env gait period", abs(gait_period - profile["gait_period_s"]) <= 1e-12)
        check(
            "saved env actuator parameters",
            all(env_actuators[name] == profile["actuators"][name] for name in joints),
        )

        xml_model_name, xml_joints = load_xml_contract(xml_snapshot)
        wholebody_model = xml_model_name in {"A00机器人全身", "jljwholebody"}
        check(
            "robot model name",
            xml_model_name == profile["robot_model"]["name"] and
            xml_model_name in {"lowbodycapsulev3", "A00机器人全身", "jljwholebody"},
        )
        check("robot XML joint set", all(name in xml_joints for name in joints))
        check(
            "robot XML joint limits",
            all(
                name in ANKLE_LIMITS
                or close_list(
                    [xml_joints[name]["lower"], xml_joints[name]["upper"]],
                    profile["joint_limits"][name],
                    tolerance=1e-12,
                )
                for name in joints
            ),
        )
        check(
            "robot XML joint axes",
            all(
                close_list(
                    xml_joints[name]["axis"],
                    profile["joint_axes"][name],
                    tolerance=1e-12,
                )
                for name in joints
            ),
        )
        if args.urdf_xacro:
            xacro_path = Path(args.urdf_xacro).expanduser().resolve()
            rendered = subprocess.run(
                ["xacro", str(xacro_path), "use_mock_hardware:=true"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
            urdf_root = ET.fromstring(rendered)
            urdf_joints = {}
            for joint in urdf_root.findall("joint"):
                if joint.attrib.get("type") != "revolute":
                    continue
                axis = joint.find("axis")
                limit = joint.find("limit")
                urdf_joints[joint.attrib["name"]] = {
                    "axis": [float(value) for value in axis.attrib["xyz"].split()],
                    "lower": float(limit.attrib["lower"]),
                    "upper": float(limit.attrib["upper"]),
                }
            check("URDF joint set", all(name in urdf_joints for name in joints))
            # The deployment URDF keeps measured hardware axes/limits for the
            # lower body and the waist parallel solver. The saved MJCF remains
            # authoritative for the policy contract, so only require the full
            # logical joint set here for whole-body profiles.
            check(
                "URDF axes match robot XML",
                wholebody_model or all(
                    close_list(
                        urdf_joints[name]["axis"], xml_joints[name]["axis"], tolerance=1e-12
                    )
                    for name in joints
                ),
            )
            check(
                "URDF limits match robot XML",
                wholebody_model or all(
                    name in ANKLE_LIMITS
                    or close_list(
                        [urdf_joints[name]["lower"], urdf_joints[name]["upper"]],
                        [xml_joints[name]["lower"], xml_joints[name]["upper"]],
                        tolerance=1e-12,
                    )
                    for name in joints
                ),
            )
            check(
                "URDF ankle limits match verified hardware ranges",
                all(
                    close_list(
                        [urdf_joints[name]["lower"], urdf_joints[name]["upper"]],
                        limits,
                        tolerance=1e-12,
                    )
                    for name, limits in ANKLE_LIMITS.items()
                ),
            )
    except (
        KeyError,
        OSError,
        TypeError,
        ValueError,
        ET.ParseError,
        subprocess.CalledProcessError,
    ) as exc:
        print(f"[FAIL] malformed or missing contract data ({exc})")
        results.append(False)

    passed = all(results) and bool(results)
    print(f"Result: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
