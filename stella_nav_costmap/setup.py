from setuptools import setup, Extension

package_name = "stella_nav_costmap"

setup(
    name=package_name,
    packages=[package_name],
    package_dir={"": "src"},
    zip_safe=False,
    entry_points={}
)

from Cython.Distutils import build_ext
import numpy

setup(
    cmdclass={"build_ext": build_ext},
    ext_modules=[
        Extension(
            "stella_nav_costmap.optimized.inflation",
            ["src/stella_nav_costmap/optimized/inflation.pyx"],
    )],
    include_dirs=[numpy.get_include()],
    zip_safe=False,
)
