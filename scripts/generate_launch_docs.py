# scripts/generate_launch_docs.py

import subprocess
from pathlib import Path

LAUNCH_DIRS = list(Path("src").rglob("launch"))  # all launch/ folders
DOCS_DIR = Path("launch_docs")
DOCS_DIR.mkdir(parents=True, exist_ok=True)

for launch_dir in LAUNCH_DIRS:
    for launch_file in launch_dir.glob("*.launch.py"):
        try:
            result = subprocess.run(
                ["ros2", "launch", str(launch_file), "--show-args"],
                capture_output=True, text=True, check=True
            )
            output = result.stdout.strip()
        except subprocess.CalledProcessError as e:
            output = f"Failed to show args for `{launch_file}`:\n{e.stderr}"

        out_path = DOCS_DIR / f"{launch_file.stem}.md"
        with open(out_path, "w") as f:
            f.write(f"# `{launch_file.name}`\n\n")
            f.write(f"**Path**: `{launch_file.relative_to(Path('.'))}`\n\n")
            f.write("```\n" + output + "\n```")
