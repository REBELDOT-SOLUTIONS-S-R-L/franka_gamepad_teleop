#!/usr/bin/env python3
"""Launch gamepad teleoperation in the existing Franka development container."""
import argparse
import json
import re
import subprocess
from pathlib import Path


def project_root(source):
    for parent in Path(source).resolve().parents:
        if (parent / "franka").is_file() and (parent / "compose.yaml").is_file():
            return parent
    raise RuntimeError("Place this repository under the Franka workspace containing franka and compose.yaml")


def main():
    root = project_root(__file__)
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    for mode in ("inspect", "simulate", "read-state", "preview", "execute", "validate-config"):
        modes.add_argument("--" + mode, dest="mode", action="store_const", const=mode)
    parser.set_defaults(mode="preview")
    parser.add_argument("--robot-ip", help="Defaults to config/robot.env in the Franka workspace")
    parser.add_argument("--config", type=Path, default=repository / "config/gamepad.yaml")
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--dry-run", action="store_true", help="Print the container command without starting it")
    args = parser.parse_args()
    try:
        config = args.config.expanduser().resolve(strict=True)
        container_config = Path("/workspace") / config.relative_to(root)
    except (OSError, ValueError) as error:
        parser.error(f"Configuration must be a file inside the Franka workspace: {error}")
    ip = args.robot_ip
    if not ip and (root / "config/robot.env").is_file():
        match = re.search(r"^ROBOT_IP=([0-9.]+)\s*$", (root / "config/robot.env").read_text(), re.M)
        if match:
            ip = match[1]
    command = [str(root / "franka"), "exec",
               "/workspace/ws/install/franka_gamepad_teleop/lib/franka_gamepad_teleop/gamepad_teleop",
               "--config", str(container_config), "--" + args.mode,
               "--duration", str(args.duration)]
    if args.mode not in ("inspect", "simulate", "validate-config"):
        if not ip:
            parser.error("Supply --robot-ip or set ROBOT_IP in config/robot.env")
        command += ["--robot-ip", ip]
    if args.dry_run:
        print(json.dumps(command, indent=2))
        return 0
    try:
        return subprocess.call(command)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
