import os
import argparse
from pathlib import Path
from jinja2 import Environment, FileSystemLoader

def find_launch_docs_for_package(launch_docs_dir: Path, package_name: str):
    """Only include .md files that correspond to launch files in this package."""
    matching_docs = []
    for md_file in launch_docs_dir.glob("*.md"):
        if md_file.name.startswith(package_name) or package_name in md_file.read_text():
            matching_docs.append({
                "name": md_file.name,
                "title": md_file.stem.replace("_", " ").title()
            })
    return sorted(matching_docs, key=lambda f: f["name"])

def main():
    parser = argparse.ArgumentParser(description="Render a markdown template with launch docs.")
    parser.add_argument("--template", required=True, help="Path to the .jinja.md template file")
    parser.add_argument("--output", required=True, help="Path to the rendered output markdown file")
    parser.add_argument("--launch-docs-dir", required=True, help="Directory containing generated .md files for launch files")
    parser.add_argument("--package-name", required=True, help="Name of the package to filter launch docs by")
    args = parser.parse_args()

    template_path = Path(args.template).resolve()
    output_path = Path(args.output).resolve()
    launch_docs_dir = Path(args.launch_docs_dir).resolve()
    package_name = args.package_name

    env = Environment(loader=FileSystemLoader(str(template_path.parent)))
    template = env.get_template(template_path.name)

    launch_docs = find_launch_docs_for_package(launch_docs_dir, package_name)

    rendered = template.render(
        repo_name=package_name,
        launch_docs=launch_docs
    )

    output_path.write_text(rendered)
    print(f"[INFO] Rendered template written to: {output_path}")

if __name__ == "__main__":
    main()
