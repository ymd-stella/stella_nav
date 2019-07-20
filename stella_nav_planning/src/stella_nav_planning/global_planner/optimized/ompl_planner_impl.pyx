#cython: language_level=2
cimport ompl
from ompl_wrapper import *
from libcpp.memory cimport shared_ptr, make_shared
from cython.operator cimport dereference as deref

class OmplPlannerImpl(object):
    def __init__(self, shape, calc_time, resolution, distance):
        self._shape = shape
        self._calc_time = calc_time
        self._space = RealVectorStateSpace(2)
        assert shape[0] == shape[1]
        self._space.setBounds(0, shape[0])
        self._si = SpaceInformation(self._space)
        self._validity_checker = ValidityChecker(self._si, shape[0], shape[1])
        self._si.setStateValidityChecker(self._validity_checker)
        self._si.setStateValidityCheckingResolution(resolution)
        self._si.setup()
        self._start = RealVectorStateType(self._space)
        self._goal = RealVectorStateType(self._space)
        self._pdef = ProblemDefinition(self._si)
        self._objective = MultiOptimizationObjective(self._si)
        self._objective.addObjective(MapCostObjective(self._si), 0.5)
        self._objective.addObjective(PathLengthOptimizationObjective(self._si), 0.1)
        self._pdef.setOptimizationObjective(self._objective)
        self._planner = RRTstar(self._si)
        self._planner.setRange(distance)
        self._planner.setup()

    def set_weight(self, w):
        self._objective = MultiOptimizationObjective(self._si)
        self._objective.addObjective(MapCostObjective(self._si), w[0])
        self._objective.addObjective(PathLengthOptimizationObjective(self._si), w[1])
        self._pdef.setOptimizationObjective(self._objective)

    def update_costmap(self, costmap):
        self._validity_checker.update_costmap(costmap.cells)

    def solve(self):
        self._pdef.setStartAndGoalStates(self._start, self._goal)
        self._planner.clear()
        self._planner.setProblemDefinition(self._pdef)
        self._planner.solve(self._calc_time)
        if self._pdef.hasExactSolution():
            states = self._pdef.getSolutionPath().getPathGeometric().getStates()
            cost = self._pdef.getSolutionPath().cost(self._objective).value()
            states_np = np.array([(state.getRealVectorState()[0], state.getRealVectorState()[1]) for state in states])
            self._pdef.clearSolutionPaths()
            return [states, cost, states_np]
        else:
            return []

    def is_stuck(self):
        return not self._validity_checker.is_valid(self._start)

    def _in_range(self, state):
        x = state[0]
        y = state[1]
        m = np.rint(np.array((x, y))).astype(np.int32)
        return 0 < x < self._shape[0] and 0 < y < self._shape[1]

    def is_unavoidable(self):
        return not self._in_range(self._goal) or not self._validity_checker.is_valid(self._goal)

    def set_problem(self, m_start, m_goal):
        self._start[0] = m_start[0]
        self._start[1] = m_start[1]
        self._goal[0] = m_goal[0]
        self._goal[1] = m_goal[1]
