#cython: language_level=3
from . cimport ompl
from ompl_wrapper import *
from libcpp.memory cimport shared_ptr, make_shared
from cython.operator cimport dereference as deref
import numpy as np

WIDTH = 100

cpdef test_rigid(calc_time=0.01):
    cells = np.zeros((WIDTH, WIDTH))
    cells[30:70, 30:70] = 1.0
    space = RealVectorStateSpace(2)
    space.setBounds(0, WIDTH)
    si = SpaceInformation(space)
    vc = ValidityChecker(si, WIDTH, WIDTH)
    vc.update_costmap(cells)
    si.setStateValidityChecker(vc)
    si.setup()
    start = RealVectorStateType(space)
    goal = RealVectorStateType(space)
    start[0] = 0.0
    start[1] = 0.0
    goal[0] = WIDTH - 1
    goal[1] = WIDTH - 1
    pdef = ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)
    planner = RRTstar(si)
    planner.setProblemDefinition(pdef)
    planner.setup()
    status = planner.solve(calc_time)
    if pdef.hasExactSolution():
        cost = pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()
        print(cost)

cpdef test_optimal(calc_time=0.01):
    cells = np.zeros((WIDTH, WIDTH))
    cells[30:70, 30:70] = 1.0
    space = RealVectorStateSpace(2)
    space.setBounds(0, WIDTH)
    si = SpaceInformation(space)
    vc = ValidityChecker(si, WIDTH, WIDTH)
    vc.update_costmap(cells)
    si.setStateValidityChecker(vc)
    si.setup()
    start = RealVectorStateType(space)
    goal = RealVectorStateType(space)
    start[0] = 0.0
    start[1] = 0.0
    goal[0] = WIDTH - 1
    goal[1] = WIDTH - 1
    pdef = ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)
    objective = MultiOptimizationObjective(si)
    objective.addObjective(MapCostObjective(si), 0.9)
    objective.addObjective(PathLengthOptimizationObjective(si), 0.1)
    pdef.setOptimizationObjective(objective)
    planner = RRTstar(si)
    planner.setProblemDefinition(pdef)
    planner.setup()
    status = planner.solve(calc_time)
    if pdef.hasExactSolution():
        states = pdef.getSolutionPath().getPathGeometric().getStates()
        cost = pdef.getSolutionPath().cost(objective).value()
        states_np = np.array([(state.getRealVectorState()[0], state.getRealVectorState()[1]) for state in states])
        print(cost)
        print(states_np)
