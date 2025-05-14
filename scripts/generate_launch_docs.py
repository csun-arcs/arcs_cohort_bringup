#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path
import sys

def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate Markdown documentation from ROS 2 launch files."
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        required=True,
        help="Path to the root of the ROS 2 workspace (e.g., ros_ws)."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional output directory for generated docs. Defaults to <workspace>/launch_docs."
    )
    parser.add_argument(
        "--package-name",
        type=str,
        required=True,
        help="Name of the package to filter launch files from."
    )
    return parser.parse_args()

def main():
    args = parse_args()
    workspace_dir = args.workspace.resolve()
    docs_dir = args.output.resolve() if args.output else workspace_dir / "launch_docs"
    docs_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Workspace root: {workspace_dir}")
    print(f"[INFO] Docs output directory: {docs_dir}")
    print(f"[INFO] Targeting package: {args.package_name}")

    package_src = workspace_dir / "src" / args.package_name
    if not package_src.exists():
        print(f"[ERROR] Package directory not found: {package_src}")
        sys.exit(1)

    launch_files = list(package_src.rglob("launch/*.launch.py"))
    if not launch_files:
        print("[WARN] No launch files found.")
    else:
        print(f"[INFO] Found {len(launch_files)} launch file(s):")
        for lf in launch_files:
            print(f" - {lf}")

    for launch_file in launch_files:
        try:
            result = subprocess.run(
                ["ros2", "launch", str(launch_file), "--show-args"],
                capture_output=True, text=True, check=True
            )
            output = result.stdout.strip()
        except subprocess.CalledProcessError as e:
            output = f"[ERROR] Failed to show args for `{launch_file}`:\n{e.stderr}"

        doc_path = docs_dir / f"{launch_file.stem}.md"
        with open(doc_path, "w") as f:
            # This comment allows filtering later by generate_template_md.py
            f.write(f"<!-- package: {args.package_name} -->\n\n")
            f.write(f"# `{launch_file.name}`\n\n")
            f.write(f"**Path**: `{launch_file.relative_to(workspace_dir)}`\n\n")
            f.write("```\n")
            f.write(output)
            f.write("\n```\n")

    print(f"[INFO] Documentation generated in: {docs_dir}")

if __name__ == "__main__":
    main()
