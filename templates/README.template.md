# {{ repo_name }}

This is the `{{ repo_name }}` ROS 2 package, part of the [CoHORT](https://github.com/csun-arcs/cohort) multi-rover autonomy software stack.

{% if description %}
## Description

{{ description }}
{% endif %}

## Documentation

👉 For full documentation and launch file reference, visit the [Wiki](https://github.com/csun-arcs/{{ repo_name }}/wiki)

{% if launch_docs %}
## Available Launch Files

The following launch files are provided by this package:

{% for file in launch_docs %}
- `{{ file.title }}` — See [{{ file.name }}](https://github.com/csun-arcs/{{ repo_name }}/wiki/{{ file.name }})
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

- 📁 GitHub: [csun-arcs/{{ repo_name }}](https://github.com/csun-arcs/{{ repo_name }})
- 📚 Wiki: [Documentation](https://github.com/csun-arcs/{{ repo_name }}/wiki)
- 👥 Contributors: [See contributors](https://github.com/csun-arcs/{{ repo_name }}/graphs/contributors)

{% if license %}
## License

This package is licensed under the **{{ license }}** license.
{% endif %}
