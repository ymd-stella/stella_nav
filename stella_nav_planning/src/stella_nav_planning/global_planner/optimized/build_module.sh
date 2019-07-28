#!/bin/bash
rm *.cpp
CFLAGS="`pkg-config --cflags eigen3` -std=c++11" \
    python3 setup.py build_ext -i \
    || echo -e "\e[31mIgnore above error about constructor.\e[m" \
    && sed -i -e '/~.*\|.*init.*/!s/\(ValidityChecker_cpp(.*)\)/\1 : ompl::base::StateValidityChecker(__pyx_v_si)/' ompl_wrapper.cpp \
    && sed -i -e '/~.*\|.*init.*/!s/\(MapCostObjective_cpp(.*)\)/\1 : ompl::base::StateCostIntegralObjective(__pyx_v_si)/' ompl_wrapper.cpp \
    && CFLAGS="`pkg-config --cflags eigen3` -std=c++11" python3 setup.py build_ext -i
