#!/usr/bin/env python3

import argparse
from pathlib import Path
from jinja2 import Environment, FileSystemLoader

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--template-dir", type=Path, required=True)
    parser.add_argument("--template-name", type=str, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--launch-docs-dir", type=Path, required=True)
    parser.add_argument("--package-name", type=str, required=True)
    return parser.parse_args()

def main():
    args = parse_args()
    env = Environment(loader=FileSystemLoader(str(args.template_dir)))
    template = env.get_template(args.template_name)

    docs_dir = args.launch_docs_dir.resolve()
    package_docs = {
        doc.stem: doc.read_text()
        for doc in docs_dir.glob("*.md")
        if doc.name.startswith(args.package_name)
    }

    output = template.render(
        package_name=args.package_name,
        launch_docs_section="\n\n".join(package_docs.values())
    )

    args.output.write_text(output)
    print(f"[INFO] Rendered template written to: {args.output}")

if __name__ == "__main__":
    main()
