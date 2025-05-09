import subprocess
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve()
SRC_DIR = SCRIPT_PATH.parents[2] / "src"  # e.g., ros_ws/src
DOCS_DIR = SCRIPT_PATH.parents[3] / "launch_docs"
DOCS_DIR.mkdir(parents=True, exist_ok=True)

print(f"[INFO] Searching for launch files in: {SRC_DIR}")

found_files = list(SRC_DIR.rglob("launch/*.launch.py"))

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
