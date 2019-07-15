from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import numpy as np

WIDTH = 100

class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super(ValidityChecker, self).__init__(si)
        self._si = si
        self._cells = np.zeros((WIDTH, WIDTH))
        self._cells[30:70, 30:70] = 1.0

    def isValid(self, state):
        x = state[0]
        y = state[1]
        return bool(self._cells[int(x), int(y)] < 0.999)

    def cost(self, state):
        x = state[0]
        y = state[1]
        return self._cells[int(x), int(y)]

class MapCostObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(MapCostObjective, self).__init__(si, True)
        self._si = si

    def stateCost(self, s):
        return ob.Cost(self._si.getStateValidityChecker().cost(s))

def test_rigid(calc_time=0.01):
    space = ob.RealVectorStateSpace(2)
    space.setBounds(0, WIDTH)
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ValidityChecker(si))
    si.setup()
    start = ob.State(space)
    goal = ob.State(space)
    start[0] = 0.0
    start[1] = 0.0
    goal[0] = WIDTH - 1
    goal[1] = WIDTH - 1
    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)
    planner = og.RRTstar(si)
    planner.setProblemDefinition(pdef)
    planner.setup()
    status = planner.solve(calc_time)
    if pdef.hasExactSolution():
        cost = pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()
        print(cost)

def test_optimal(calc_time=0.01):
    space = ob.RealVectorStateSpace(2)
    space.setBounds(0, WIDTH)
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ValidityChecker(si))
    si.setup()
    start = ob.State(space)
    goal = ob.State(space)
    start[0] = 0.0
    start[1] = 0.0
    goal[0] = WIDTH - 1
    goal[1] = WIDTH - 1
    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)
    objective = ob.MultiOptimizationObjective(si)
    objective.addObjective(MapCostObjective(si), 0.9)
    objective.addObjective(ob.PathLengthOptimizationObjective(si), 0.1)
    pdef.setOptimizationObjective(objective)
    planner = og.RRTstar(si)
    planner.setProblemDefinition(pdef)
    planner.setup()
    status = planner.solve(calc_time)
    if pdef.hasExactSolution():
        states = pdef.getSolutionPath().getStates()
        cost = pdef.getSolutionPath().cost(objective).value()
        states_np = np.array([(state[0], state[1]) for state in states])
        print(states)
        print(cost)
        print(states_np)
