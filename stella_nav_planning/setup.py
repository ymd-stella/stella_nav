from setuptools import setup, Extension

package_name = "stella_nav_planning"

setup(
    name=package_name,
    packages=[package_name],
    package_dir={"": "src"},
    zip_safe=False,
    entry_points={}
)

from Cython.Distutils import build_ext
import pkgconfig

ext_modules = [
        Extension(
            "stella_nav_planning.global_planner.optimized.ompl_wrapper",
            ["src/stella_nav_planning/global_planner/optimized/ompl_wrapper.pyx"],
            include_dirs=[pkgconfig.cflags("eigen3").lstrip("-I")],
            extra_compile_args=["-std=c++11"],
            libraries=["ompl"],
            language="c++"),
        Extension(
            "stella_nav_planning.global_planner.optimized.ompl_planner_impl",
            ["src/stella_nav_planning/global_planner/optimized/ompl_planner_impl.pyx"],
            include_dirs=[pkgconfig.cflags("eigen3").lstrip("-I")],
            extra_compile_args=["-std=c++11"],
            libraries=["ompl"],
            language="c++"),
        Extension(
            "stella_nav_planning.global_planner.optimized.test_planning",
            ["src/stella_nav_planning/global_planner/optimized/test_planning.pyx"],
            include_dirs=[pkgconfig.cflags("eigen3").lstrip("-I")],
            extra_compile_args=["-std=c++11"],
            libraries=["ompl"],
            language="c++")]

import os, re, fileinput
rep_not = re.compile(".*~.*|.*init.*|.*cppclass.*")
rep1 = re.compile("(ValidityChecker_cpp\(.*\))")
to1 = "\1 : ompl::base::StateValidityChecker(__pyx_v_si)"
rep2 = re.compile("(MapCostObjective_cpp\(.*\))")
to2 = "\1 : ompl::base::StateCostIntegralObjective(__pyx_v_si)"
class work_around_build_ext(build_ext):
    def _replace(self, path):
        f = fileinput.input(path, inplace=True)
        for line in f:
            if rep_not.match(line) is not None:
                continue
            tmp = rep1.sub(to1, line)
            line = rep2.sub(to2, tmp)
        f.close()

    def build_extensions(self):
        self.check_extensions_list(self.extensions)
        if os.path.exists("src/stella_nav_planning/global_planner/optimized/ompl_wrapper.cpp"):
            os.remove("src/stella_nav_planning/global_planner/optimized/ompl_wrapper.cpp")

        for ext in self.extensions:
            ext.sources = self.cython_sources(ext.sources, ext)
            if ext.sources[0].endswith("ompl_wrapper.cpp"):
                self._replace(ext.sources[0])
            self.build_extension(ext)

setup(
    cmdclass={"build_ext": work_around_build_ext},
    ext_modules = ext_modules,
    zip_safe=False
)
