import subprocess
from pathlib import Path

# Locate the script and workspace root
SCRIPT_PATH = Path(__file__).resolve()
WORKSPACE_DIR = SCRIPT_PATH.parents[4]  # ros_ws
DOCS_DIR = WORKSPACE_DIR / "launch_docs"
DOCS_DIR.mkdir(parents=True, exist_ok=True)

print(f"[INFO] Script path: {SCRIPT_PATH}")
print(f"[INFO] Workspace root: {WORKSPACE_DIR}")
print(f"[INFO] Docs output directory: {DOCS_DIR}")

# Recursively find all *.launch.py files under any launch/ directory
found_files = list(WORKSPACE_DIR.rglob("launch/*.launch.py"))

if not found_files:
    print("[WARN] No launch files found.")
else:
    print(f"[INFO] Found {len(found_files)} launch file(s):")
    for lf in found_files:
        print(f" - {lf}")

# Run ros2 launch --show-args on each file and dump to Markdown
for launch_file in found_files:
    try:
        result = subprocess.run(
            ["ros2", "launch", str(launch_file), "--show-args"],
            capture_output=True, text=True, check=True
        )
        output = result.stdout.strip()
    except subprocess.CalledProcessError as e:
        output = f"[ERROR] Failed to show args for `{launch_file}`:\n{e.stderr}"

    doc_path = DOCS_DIR / f"{launch_file.stem}.md"
    with open(doc_path, "w") as f:
        f.write(f"# `{launch_file.name}`\n\n")
        f.write(f"**Path**: `{launch_file.relative_to(WORKSPACE_DIR)}`\n\n")
        f.write("```\n")
        f.write(output)
        f.write("\n```")

print(f"[INFO] Documentation generated in: {DOCS_DIR}")
