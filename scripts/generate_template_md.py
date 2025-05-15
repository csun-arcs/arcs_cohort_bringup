#!/usr/bin/env python3

import argparse
from pathlib import Path

from jinja2 import Environment, FileSystemLoader


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render a Jinja2 template for Home.md with launch docs."
    )
    parser.add_argument(
        "--template-dir",
        type=Path,
        required=True,
        help="Directory containing the Jinja2 template.",
    )
    parser.add_argument(
        "--template-name",
        type=str,
        required=True,
        help="Name of the Jinja2 template file (e.g. Home.template.md).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Path to write the rendered file (e.g. wiki/Home.md).",
    )
    parser.add_argument(
        "--launch-docs-dir",
        type=Path,
        required=True,
        help="Base directory containing per-package .md launch files.",
    )
    parser.add_argument(
        "--package-name",
        type=str,
        required=True,
        help="Name of the package to filter launch docs for.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="If set, renders the output to stdout instead of writing to a file.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    env = Environment(loader=FileSystemLoader(str(args.template_dir)))
    template = env.get_template(args.template_name)

    package_doc_dir = args.launch_docs_dir / args.package_name
    if not package_doc_dir.exists():
        print(f"[WARN] Launch doc directory not found: {package_doc_dir}")
        launch_docs = []
    else:
        launch_docs = [
            {
                "name": md.name,
                "title": md.name.removesuffix(".launch.md").replace("_", " ").title(),
            }
            for md in sorted(package_doc_dir.glob("*.md"))
        ]

    if not launch_docs:
        print(f"[WARN] No launch docs found for package: {args.package_name}")
    else:
        print(
            f"[INFO] Found {len(launch_docs)} launch docs for package: {args.package_name}"
        )

    context = {
        "repo_name": args.package_name,
        "launch_docs": launch_docs,
    }

    rendered = template.render(**context)

    if args.dry_run:
        print("[INFO] Dry run enabled - rendered output below:")
        print("=" * 40)
        print(rendered)
        print("=" * 40)
    else:
        args.output.write_text(rendered)
        print(f"[INFO] Rendered template written to: {args.output}")


if __name__ == "__main__":
    main()
