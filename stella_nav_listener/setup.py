from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["stella_nav_listener"],
    package_dir={"":"src"},
)

setup(**setup_args)
