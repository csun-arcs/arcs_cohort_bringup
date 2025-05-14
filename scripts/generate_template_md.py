#!/usr/bin/env python3

import argparse
from pathlib import Path
from jinja2 import Environment, FileSystemLoader

def parse_args():
    parser = argparse.ArgumentParser(description="Render a Jinja2 template for Home.md with launch docs.")
    parser.add_argument("--template-dir", type=Path, required=True, help="Directory containing the Jinja2 template.")
    parser.add_argument("--template-name", type=str, required=True, help="Name of the Jinja2 template file.")
    parser.add_argument("--output", type=Path, required=True, help="Path to output rendered file (e.g. wiki/Home.md).")
    parser.add_argument("--launch-docs-dir", type=Path, required=True, help="Directory containing .md launch files.")
    parser.add_argument("--package-name", type=str, required=True, help="Name of the package to match launch docs for.")
    return parser.parse_args()

def parse_package_comment(md_file: Path) -> str:
    try:
        with md_file.open() as f:
            for line in f:
                if line.strip().startswith("<!-- package:"):
                    return line.strip().split("package:")[1].strip(" -->")
    except Exception as e:
        print(f"[WARN] Failed to read {md_file}: {e}")
    return None

def main():
    args = parse_args()

    env = Environment(loader=FileSystemLoader(str(args.template_dir)))
    template = env.get_template(args.template_name)

    launch_docs = []
    for md_file in sorted(args.launch_docs_dir.glob("*.md")):
        pkg = parse_package_comment(md_file)
        if pkg == args.package_name:
            launch_docs.append({
                "name": md_file.name,
                "title": md_file.stem.replace("_", " ").capitalize()
            })

    if not launch_docs:
        print(f"[WARN] No launch docs found for package: {args.package_name}")
    else:
        print(f"[INFO] Found {len(launch_docs)} launch docs for package: {args.package_name}")

    context = {
        "repo_name": args.package_name,
        "launch_docs": launch_docs,
    }

    args.output.write_text(template.render(**context))
    print(f"[INFO] Rendered template written to: {args.output}")

if __name__ == "__main__":
    main()
