from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
import numpy as np


class OmplPlannerImpl(object):
    class ValidityChecker(ob.StateValidityChecker):
        def __init__(self, si):
            super(OmplPlannerImpl.ValidityChecker, self).__init__(si)
            self._si = si
            self._costmap = None

        def isValid(self, state):
            x = state[0]
            y = state[1]
            m = np.rint(np.array((x, y))).astype(np.int32)
            return bool(self._costmap.get_value(m) < 0.999)

        def isRange(self, state):
            x = state[0]
            y = state[1]
            m = np.rint(np.array((x, y))).astype(np.int32)
            return bool(self._costmap.is_contained(m))

        def cost(self, state):
            x = state[0]
            y = state[1]
            m = np.rint(np.array((x, y))).astype(np.int32)
            return self._costmap.get_value(m)

        def update_costmap(self, costmap):
            self._costmap = costmap

    class MapCostObjective(ob.StateCostIntegralObjective):
        def __init__(self, si):
            super(OmplPlannerImpl.MapCostObjective, self).__init__(si, True)
            self._si = si

        def stateCost(self, s):
            return ob.Cost(self._si.getStateValidityChecker().cost(s))

    def __init__(self, shape, calc_time, resolution, distance):
        ou.setLogLevel(ou.LOG_WARN)
        self._calc_time = calc_time
        self._space = ob.RealVectorStateSpace(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0.0)
        bounds.setHigh(0, shape[0]-1)
        bounds.setHigh(1, shape[1]-1)
        self._space.setBounds(bounds)
        self._si = ob.SpaceInformation(self._space)
        self._validity_checker = OmplPlannerImpl.ValidityChecker(self._si)
        self._si.setStateValidityChecker(self._validity_checker)
        self._si.setStateValidityCheckingResolution(resolution)
        self._si.setup()
        self._start = ob.State(self._space)
        self._goal = ob.State(self._space)
        self._pdef = ob.ProblemDefinition(self._si)
        objective = ob.MultiOptimizationObjective(self._si)
        objective.addObjective(OmplPlannerImpl.MapCostObjective(self._si), 0.5)
        # ## setting length objective
        length_objective = ob.PathLengthOptimizationObjective(self._si)
        objective.addObjective(length_objective, 0.1)
        # objective.setCostThreshold(1000.0)
        self._pdef.setOptimizationObjective(objective)
        self._planner = og.RRTstar(self._si)
        self._planner.setRange(distance)
        self._planner.setup()

    def set_weight(self, w):
        objective = ob.MultiOptimizationObjective(self._si)
        objective.addObjective(OmplPlannerImpl.MapCostObjective(self._si), w[0])
        objective.addObjective(ob.PathLengthOptimizationObjective(self._si), w[1])
        self._pdef.setOptimizationObjective(objective)

    def update_costmap(self, costmap):
        self._validity_checker.update_costmap(costmap)

    def solve(self):
        self._pdef.setStartAndGoalStates(self._start, self._goal)
        self._planner.clear()
        self._planner.setProblemDefinition(self._pdef)
        self._planner.solve(self._calc_time)
        if self._pdef.hasExactSolution():
            states = self._pdef.getSolutionPath().getStates()
            cost = self._pdef.getSolutionPath().cost(self._pdef.getOptimizationObjective()).value()
            states_np = np.array([(state[0], state[1]) for state in states])
            self._pdef.clearSolutionPaths()
            return [states, cost, states_np]
        else:
            return []

    def is_stuck(self):
        return not self._validity_checker.isValid(self._start)

    def is_unavoidable(self):
        return not self._validity_checker.isRange(self._goal) or not self._validity_checker.isValid(self._goal)

    def set_problem(self, m_start, m_goal):
        self._start[0] = m_start[0]
        self._start[1] = m_start[1]
        self._goal[0] = m_goal[0]
        self._goal[1] = m_goal[1]
