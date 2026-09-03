#!/usr/bin/env python3
"""Prepare and optionally activate a deployment profile for one mjlab run."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import yaml


def discover_model(run_dir: Path) -> Path:
    named_export = run_dir / f"{run_dir.name}.onnx"
    if named_export.is_file():
        return named_export
    exports = sorted(run_dir.glob("*.onnx"))
    if len(exports) == 1:
        return exports[0]
    if not exports:
        raise ValueError(f"no ONNX export found directly under {run_dir}")
    raise ValueError(
        "multiple ONNX exports found; select one with --model: "
        + ", ".join(path.name for path in exports)
    )


def discover_robot_xml(run_dir: Path) -> Path:
    try:
        mjlab_root = run_dir.parents[3]
    except IndexError as exc:
        raise ValueError("cannot infer the mjlab root from the run path") from exc
    candidates = (
        mjlab_root
        / "src/assets/robots/jljwhole/A00机器人全身_mjcf/A00机器人全身/A00机器人全身.xml",
        mjlab_root / "src/assets/robots/jljbot/lowbodycapsulev3.xml",
        mjlab_root / "src/mjlab/asset_zoo/robots/jljbot/xml/lowbodycapsulev3.xml",
    )
    for xml_path in candidates:
        if xml_path.is_file():
            return xml_path
    raise ValueError(
        "inferred robot XML does not exist: "
        + ", ".join(str(path) for path in candidates)
    )


def run_checked(command: list[str]) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--model", default="", help="Override automatic ONNX discovery")
    parser.add_argument("--robot-xml", default="", help="Override automatic robot XML discovery")
    parser.add_argument("--profile-name", default="", help="Default: training run directory name")
    parser.add_argument(
        "--activate",
        action="store_true",
        help="Replace policies/active.yaml after generation and validation",
    )
    parser.add_argument(
        "--runtime-dir",
        default="",
        help="Runtime policy store used by deploy.yaml (default: package config/policies)",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    package_dir = script_dir.parent
    workspace_dir = package_dir.parents[1]
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not (run_dir / "params/env.yaml").is_file():
        print(f"FAIL: run has no params/env.yaml: {run_dir}")
        return 2

    try:
        model_path = (
            Path(args.model).expanduser().resolve()
            if args.model
            else discover_model(run_dir)
        )
        xml_path = (
            Path(args.robot_xml).expanduser().resolve()
            if args.robot_xml
            else discover_robot_xml(run_dir)
        )
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2

    profile_name = args.profile_name or run_dir.name
    if Path(profile_name).name != profile_name or profile_name in {"active", ".", ".."}:
        print("FAIL: profile name must be a simple non-reserved filename stem")
        return 2
    deploy_config = package_dir / "config/deploy.yaml"
    runtime_dir = (
        Path(args.runtime_dir).expanduser().resolve()
        if args.runtime_dir
        else package_dir / "config" / "policies"
    )
    urdf_xacro = (
        workspace_dir
        / "src/jljwholebody_description/urdf/jljwholebody.urdf.xacro"
    )
    staging_dir = None
    candidate_path = None
    if args.activate:
        runtime_dir.mkdir(parents=True, exist_ok=True)
        staging_dir = tempfile.TemporaryDirectory(
            prefix=f".{profile_name}.staging-", dir=runtime_dir
        )
        output_path = Path(staging_dir.name) / f"{profile_name}.yaml"
    else:
        output_path = package_dir / "config/policies" / f"{profile_name}.yaml"

    try:
        run_checked(
            [
                sys.executable,
                str(script_dir / "generate_policy_profile.py"),
                "--run-dir",
                str(run_dir),
                "--model",
                str(model_path),
                "--robot-xml",
                str(xml_path),
                "--output",
                str(output_path),
            ]
        )
        validate_command = [
            sys.executable,
            str(script_dir / "check_policy_contract.py"),
            "--config",
            str(deploy_config),
            "--profile",
            str(output_path),
        ]
        if urdf_xacro.is_file():
            validate_command.extend(["--urdf-xacro", str(urdf_xacro)])
        run_checked(validate_command)

        generated_profile = yaml.safe_load(output_path.read_text())["policy_profile"]
        generated_model = (output_path.parent / generated_profile["model_path"]).resolve()
        if args.activate:
            bundle_dir = runtime_dir / profile_name
            bundle_dir.mkdir(parents=True, exist_ok=True)
            runtime_model = bundle_dir / generated_model.name
            runtime_env = bundle_dir / "env.yaml"
            xml_snapshot = (
                output_path.parent
                / generated_profile["robot_model"]["xml_snapshot"]
            ).resolve()
            runtime_xml = bundle_dir / xml_snapshot.name
            shutil.copy2(generated_model, runtime_model)
            shutil.copy2(run_dir / "params/env.yaml", runtime_env)
            shutil.copy2(xml_snapshot, runtime_xml)

            active_document = yaml.safe_load(output_path.read_text())
            active_profile = active_document["policy_profile"]
            active_profile["source"]["env_yaml"] = runtime_env.name
            active_profile["source"]["original_onnx"] = runtime_model.name
            active_profile["source"]["original_robot_xml"] = runtime_xml.name
            active_profile["model_path"] = runtime_model.name
            active_profile["robot_model"]["xml_snapshot"] = runtime_xml.name
            runtime_dir.mkdir(parents=True, exist_ok=True)
            profile_text = (
                "# Managed by prepare_policy_deployment.py; do not edit.\n"
                + yaml.safe_dump(active_document, sort_keys=False)
            )
            profile_path = bundle_dir / "profile.yaml"
            profile_path.write_text(profile_text, encoding="utf-8")

            # active.yaml is intentionally only a selector. This keeps each
            # generated profile immutable and makes manual policy comparison a
            # one-line edit without duplicating the full contract.
            selector_document = {
                "schema_version": 1,
                "active_profile": os.path.relpath(profile_path, runtime_dir),
            }
            selector_text = (
                "# Select the generated policy profile used by deployment.\n"
                "# This file is safe to edit manually; paths are relative to this file.\n"
                + yaml.safe_dump(selector_document, sort_keys=False)
            )
            candidate_path = runtime_dir / "active.yaml.candidate"
            candidate_path.write_text(selector_text, encoding="utf-8")
            candidate_validate = [
                sys.executable,
                str(script_dir / "check_policy_contract.py"),
                "--config",
                str(deploy_config),
                "--profile",
                str(candidate_path),
            ]
            if urdf_xacro.is_file():
                candidate_validate.extend(["--urdf-xacro", str(urdf_xacro)])
            run_checked(candidate_validate)
            active_path = runtime_dir / "active.yaml"
            candidate_path.replace(active_path)
            print(f"ACTIVE: {profile_name}")
            print(f"Runtime profile: {active_path}")
        else:
            print(f"READY: {output_path}")
            print("Not activated. Re-run with --activate after reviewing the result.")
    except subprocess.CalledProcessError as exc:
        print(f"FAIL: command exited with status {exc.returncode}")
        return exc.returncode or 1
    finally:
        if candidate_path is not None:
            candidate_path.unlink(missing_ok=True)
        if staging_dir is not None:
            staging_dir.cleanup()

    if args.activate:
        print("Next: rebuild once if the native deploy package changed, then restart the node.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
