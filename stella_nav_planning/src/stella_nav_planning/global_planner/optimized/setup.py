from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

setup(
    ext_modules = cythonize(
        [Extension(
            "ompl_wrapper",
            ["ompl_wrapper.pyx"],
            libraries=["ompl"],
            language="c++"),
        Extension(
            "ompl_planner_impl",
            ["ompl_planner_impl.pyx"],
            libraries=["ompl"],
            language="c++"),
        Extension(
            "test_planning",
            ["test_planning.pyx"],
            libraries=["ompl"],
            language="c++")
        ])
)
