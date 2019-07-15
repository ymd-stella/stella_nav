cimport ompl
from libcpp.memory cimport shared_ptr, make_shared, static_pointer_cast
from libcpp.vector cimport vector
from libc.math cimport sqrt
cimport libcpp
import numpy as np
cimport numpy as cnp
from libc.stdlib cimport malloc
from cython.operator cimport dereference as deref

cdef class StateSpace:
    cdef shared_ptr[ompl.StateSpace] _thisptr

cdef class RealVectorStateSpace(StateSpace):
    def __cinit__(self, unsigned int dim):
        self._thisptr = <shared_ptr[ompl.StateSpace]>make_shared[ompl.RealVectorStateSpace](dim)

    def setBounds(self, double low, double high):
        deref(static_pointer_cast[ompl.RealVectorStateSpace, ompl.StateSpace](self._thisptr)).setBounds(low, high)

cdef class SpaceInformation:
    cdef shared_ptr[ompl.SpaceInformation] _thisptr
    def __cinit__(self, StateSpace space):
        self._thisptr = make_shared[ompl.SpaceInformation](space._thisptr)

    def setStateValidityChecker(self, StateValidityChecker svc):
        deref(self._thisptr).setStateValidityChecker(svc._thisptr)

    def setStateValidityCheckingResolution(self, double resolution):
        deref(self._thisptr).setStateValidityCheckingResolution(resolution)

    def setup(self):
        deref(self._thisptr).setup()

cdef class ProblemDefinition:
    cdef shared_ptr[ompl.ProblemDefinition] _thisptr
    def __cinit__(self, SpaceInformation si):
        self._thisptr = make_shared[ompl.ProblemDefinition](si._thisptr)

    def setStartAndGoalStates(self, RealVectorStateType start, RealVectorStateType goal):
        deref(self._thisptr).setStartAndGoalStates(start._thisptr, goal._thisptr)

    def hasExactSolution(self):
        return deref(self._thisptr).hasExactSolution()

    def getSolutionPath(self):
        return Path.factory_ptr(deref(self._thisptr).getSolutionPath())

    def getOptimizationObjective(self):
        return OptimizationObjective.factory_ptr(deref(self._thisptr).getOptimizationObjective())

    def setOptimizationObjective(self, OptimizationObjective objective):
        deref(self._thisptr).setOptimizationObjective(objective._thisptr)

    def clearSolutionPaths(self):
        deref(self._thisptr).clearSolutionPaths()

cdef class RealVectorStateType(State):
    cdef shared_ptr[ompl.RealVectorStateSpace] _space
    def __cinit__(self, RealVectorStateSpace space):
        self._space = static_pointer_cast[ompl.RealVectorStateSpace, ompl.StateSpace](space._thisptr)
        self._thisptr = deref(self._space).allocState()

    def __dealloc__(self):
        if self._thisptr != NULL:
            deref(self._space).freeState(<ompl.State*>self._thisptr)

    def __getitem__(self, unsigned int i):
        return deref(<ompl.RealVectorStateType*>self._thisptr)[i]

    def __setitem__(self, unsigned int i, double value):
        deref(<ompl.RealVectorStateType*>self._thisptr)[i] = value

cdef class RealVectorStateTypeRef(State):
    def __cinit__(self):
        pass

    def __getitem__(self, unsigned int i):
        return deref(<ompl.RealVectorStateType*>self._thisptr)[i]

    def __setitem__(self, unsigned int i, double value):
        deref(<ompl.RealVectorStateType*>self._thisptr)[i] = value

    @staticmethod
    cdef factory_state_raw_ptr(ompl.State* state):
        obj = RealVectorStateTypeRef()
        obj._thisptr = state
        return obj

cdef class PlannerStatus:
    cdef shared_ptr[ompl.PlannerStatus] _thisptr
    def __cinit__(self):
        pass

    @staticmethod
    cdef factory(ompl.PlannerStatus status):
        obj = PlannerStatus()
        obj._thisptr = make_shared[ompl.PlannerStatus](status)
        return obj

cdef class Planner:
    cdef shared_ptr[ompl.Planner] _thisptr

cdef class RRTstar(Planner):
    def __cinit__(self, SpaceInformation si):
        self._thisptr = <shared_ptr[ompl.Planner]>make_shared[ompl.RRTstar](si._thisptr)

    def setProblemDefinition(self, ProblemDefinition pdef):
        deref(static_pointer_cast[ompl.RRTstar, ompl.Planner](self._thisptr)).setProblemDefinition(pdef._thisptr)

    def setup(self):
        deref(static_pointer_cast[ompl.RRTstar, ompl.Planner](self._thisptr)).setup()

    def solve(self, double solveTime):
        return PlannerStatus.factory(deref(self._thisptr).solve(solveTime))

    def setRange(self, double distance):
        deref(static_pointer_cast[ompl.RRTstar, ompl.Planner](self._thisptr)).setRange(distance)

    def clear(self):
        deref(static_pointer_cast[ompl.RRTstar, ompl.Planner](self._thisptr)).clear()

cdef class StateValidityChecker:
    cdef shared_ptr[ompl.StateValidityChecker] _thisptr

cdef cppclass ValidityChecker_cpp(ompl.StateValidityChecker):
    double* _cells
    int rows
    int cols
    __init__(shared_ptr[ompl.SpaceInformation] si, int rows, int cols):
        # WORK_AROUND: cannot call parent class constructor
        # ref: https://github.com/cython/cython/issues/1245
        # need to edit ompl_wrapper.cpp (READ build_module.sh)
        this.rows = rows
        this.cols = cols
        this._cells = <double*>malloc(this.rows * this.cols * sizeof(double))

    libcpp.bool isValid(const ompl.State* state) const:
        cdef double[:, :] mv
        mv = <double[:this.rows, :this.cols]>this._cells
        cdef ompl.RealVectorStateType* state2D
        state2D = <ompl.RealVectorStateType*>state
        cdef double x = deref(state2D)[0]
        cdef double y = deref(state2D)[1]
        return mv[<int>x, <int>y] < 0.999

    ompl.Cost cost(const ompl.State* state) const:
        cdef double[:, :] mv
        mv = <double[:this.rows, :this.cols]>this._cells
        cdef ompl.RealVectorStateType* state2D
        state2D = <ompl.RealVectorStateType*>state
        cdef double x = deref(state2D)[0]
        cdef double y = deref(state2D)[1]
        return ompl.Cost(mv[<int>x, <int>y])

    void updateCostmap(double [:, :] cells):
        cdef double[:, :] mv
        mv = <double[:this.rows, :this.cols]>this._cells
        mv[:, :] = cells

cdef cppclass MapCostObjective_cpp(ompl.StateCostIntegralObjective):
    shared_ptr[ompl.SpaceInformation] si
    MapCostObjective_cpp(shared_ptr[ompl.SpaceInformation] si):
        # WORK_AROUND: cannot call parent class constructor
        # ref: https://github.com/cython/cython/issues/1245
        # need to edit ompl_wrapper.cpp (READ build_module.sh)
        this.si = si
    ompl.Cost stateCost(const ompl.State* s) const:
        return deref(static_pointer_cast[ValidityChecker_cpp, ompl.StateValidityChecker](deref(this.si).getStateValidityChecker())).cost(s)

cdef class ValidityChecker(StateValidityChecker):
    def __cinit__(self, SpaceInformation si, int rows, int cols):
        self._thisptr = <shared_ptr[ompl.StateValidityChecker]>make_shared[ValidityChecker_cpp](si._thisptr, rows, cols)
    def update_costmap(self, double [:, :] cells):
        deref(static_pointer_cast[ValidityChecker_cpp, ompl.StateValidityChecker](self._thisptr)).updateCostmap(cells)
    def is_valid(self, State state):
        return deref(static_pointer_cast[ValidityChecker_cpp, ompl.StateValidityChecker](self._thisptr)).isValid(state._thisptr)

cdef class OptimizationObjective:
    cdef shared_ptr[ompl.OptimizationObjective] _thisptr
    def __cinit__(self):
        pass

    @staticmethod
    cdef factory_ptr(shared_ptr[ompl.OptimizationObjective] objective):
        obj = OptimizationObjective()
        obj._thisptr = objective
        return obj

cdef class MultiOptimizationObjective(OptimizationObjective):
    def __cinit__(self, SpaceInformation si):
        self._thisptr = <shared_ptr[ompl.OptimizationObjective]>make_shared[ompl.MultiOptimizationObjective](si._thisptr)
    def addObjective(self, OptimizationObjective objective, double weight):
        deref(static_pointer_cast[ompl.MultiOptimizationObjective, ompl.OptimizationObjective](self._thisptr)).addObjective(objective._thisptr, weight)

cdef class PathLengthOptimizationObjective(OptimizationObjective):
    def __cinit__(self, SpaceInformation si):
        self._thisptr = <shared_ptr[ompl.OptimizationObjective]>make_shared[ompl.PathLengthOptimizationObjective](si._thisptr)

cdef class MapCostObjective(OptimizationObjective):
    def __cinit__(self, SpaceInformation si):
        self._thisptr = <shared_ptr[ompl.OptimizationObjective]>make_shared[MapCostObjective_cpp](si._thisptr)

cdef class Cost:
    cdef shared_ptr[ompl.Cost] _thisptr
    def __cinit__(self):
        pass

    def value(self):
        return deref(self._thisptr).value()

    @staticmethod
    cdef factory(ompl.Cost cost):
        obj = Cost()
        obj._thisptr = make_shared[ompl.Cost](cost)
        return obj

cdef class Path:
    cdef shared_ptr[ompl.Path] _thisptr
    def __cinit__(self):
        pass

    def cost(self, OptimizationObjective obj):
        return Cost.factory(deref(self._thisptr).cost(obj._thisptr))

    def getPathGeometric(self):
        return PathGeometric.factory_ptr(self._thisptr)

    @staticmethod
    cdef factory_ptr(shared_ptr[ompl.Path] path):
        obj = Path()
        obj._thisptr = path
        return obj

cdef class State:
    cdef ompl.State* _thisptr
    @staticmethod
    cdef factory_raw_ptr(ompl.State* state):
        obj = State()
        obj._thisptr = state
        return obj

    def getRealVectorState(self):
        return RealVectorStateTypeRef.factory_state_raw_ptr(self._thisptr)

cdef class PathGeometric(Path):
    def __cinit__(self):
        pass

    def getStates(self):
        cdef vector[ompl.State*] states = deref(static_pointer_cast[ompl.PathGeometric, ompl.Path](self._thisptr)).getStates()
        return [State.factory_raw_ptr(state) for state in states]

    @staticmethod
    cdef factory_ptr(shared_ptr[ompl.Path] path):
        obj = PathGeometric()
        obj._thisptr = path
        return obj
