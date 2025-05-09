import subprocess
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve()
WORKSPACE_DIR = SCRIPT_PATH.parents[4]  # ros_ws
found_files = list(WORKSPACE_DIR.rglob("launch/*.launch.py"))
DOCS_DIR = WORKSPACE_DIR / "launch_docs"
DOCS_DIR.mkdir(parents=True, exist_ok=True)

print(f"[INFO] Searching for launch files in: {WORKSPACE_DIR}")

if not found_files:
    print("[WARN] No launch files found.")
else:
    print(f"[INFO] Found {len(found_files)} launch file(s):")
    for lf in found_files:
        print(f" - {lf}")

for launch_file in found_files:
    try:
        result = subprocess.run(
            ["ros2", "launch", str(launch_file), "--show-args"],
            capture_output=True, text=True, check=True
        )
        output = result.stdout.strip()
    except subprocess.CalledProcessError as e:
        output = f"[ERROR] Failed to show args for `{launch_file}`:\n{e.stderr}"

    out_path = DOCS_DIR / f"{launch_file.stem}.md"
    with open(out_path, "w") as f:
        f.write(f"# `{launch_file.name}`\n\n")
        f.write(f"**Path**: `{launch_file.relative_to(SRC_DIR)}`\n\n")
        f.write("```\n" + output + "\n```")
