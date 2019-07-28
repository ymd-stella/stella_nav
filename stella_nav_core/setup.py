from setuptools import setup

package_name = "stella_nav_core"

setup(
    name=package_name,
    packages=[package_name],
    package_dir={"": "src"},
    entry_points={
        "console_scripts": [
            "ros2_debug_node = stella_nav_core.ros2_debug_node:main"
        ]
    },
    zip_safe=False
)
