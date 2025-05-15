# {{ repo_name }}

![License](https://img.shields.io/github/license/{{ github_user }}/{{ repo_name }})
![Last Commit](https://img.shields.io/github/last-commit/{{ github_user }}/{{ repo_name }})

This is the `{{ repo_name }}` ROS 2 package, part of the [CoHORT](https://github.com/{{ github_user }}/arcs_cohort) multi-rover autonomy software stack.

{% if description %}
## Description

{{ description }}
{% endif %}

## 📦 Build Status

| Branch | Docs | Tests |
|--------|------|-------|
{% if 'main' in branches %}
| `main` | ![Docs](https://github.com/{{ repo_name }}/actions/workflows/generate-docs.yml/badge.svg?branch=main) | ![Tests](https://github.com/{{ repo_name }}/actions/workflows/run-tests.yml/badge.svg?branch=main) |
{% endif %}
{% if 'jazzy' in branches %}
| `jazzy` | ![Docs](https://github.com/{{ repo_name }}/actions/workflows/generate-docs.yml/badge.svg?branch=jazzy) | ![Tests](https://github.com/{{ repo_name }}/actions/workflows/run-tests.yml/badge.svg?branch=jazzy) |
{% endif %}
{% if 'humble' in branches %}
| `humble` | ![Docs](https://github.com/{{ repo_name }}/actions/workflows/generate-docs.yml/badge.svg?branch=humble) | ![Tests](https://github.com/{{ repo_name }}/actions/workflows/run-tests.yml/badge.svg?branch=humble) |
{% endif %}

## Documentation

👉 For full documentation and launch file reference, visit the [Wiki](https://github.com/{{ github_user }}/{{ repo_name }}/wiki)

{% if launch_docs %}
## Available Launch Files

The following launch files are provided by this package:

{% for file in launch_docs %}
- `{{ file.title }}` — See [{{ file.name }}](https://github.com/{{ github_user }}/{{ repo_name }}/wiki/{{ file.name }})
{% endfor %}
{% else %}
_This package does not contain any launch files._
{% endif %}

{% if maintainers %}
## Maintainers

{% for maint in maintainers %}
- {{ maint.name }}{% if maint.obfuscated_email %} ({{ maint.obfuscated_email }}){% endif %}
{% endfor %}
{% endif %}

## Repository

- 📁 GitHub: [{{ github_user }}/{{ repo_name }}](https://github.com/{{ github_user }}/{{ repo_name }})
- 📚 Wiki: [Documentation](https://github.com/{{ github_user }}/{{ repo_name }}/wiki)
- 👥 Contributors: [See contributors](https://github.com/{{ github_user }}/{{ repo_name }}/graphs/contributors)

{% if license %}
## License

This package is licensed under the **{{ license }}** license.
{% endif %}
