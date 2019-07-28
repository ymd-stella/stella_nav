from setuptools import setup, Extension

package_name = "stella_nav_handler"

setup(
    name=package_name,
    packages=[package_name],
    package_dir={"": "src"},
    zip_safe=False,
    entry_points={}
)
