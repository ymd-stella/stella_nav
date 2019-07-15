from libcpp.memory cimport shared_ptr
from libcpp.vector cimport vector
cimport libcpp
cdef extern from "ompl/base/State.h" namespace "ompl::base":
    cdef cppclass StateSpace:
        pass
    cdef cppclass State:
        pass

cdef extern from "ompl/base/spaces/RealVectorStateSpace.h" namespace "ompl::base":
    cdef cppclass RealVectorStateSpace(StateSpace):
        cppclass StateType(State):
            StateType()
            double& operator[](unsigned int)
        RealVectorStateSpace(unsigned int dim)
        void setBounds(double low, double high)
        State* allocState()
        void freeState(State* state)
# WORK_AROUND: cannot refer nested type
ctypedef RealVectorStateSpace.StateType RealVectorStateType

cdef extern from "ompl/base/SpaceInformation.h" namespace "ompl::base":
    cdef cppclass SpaceInformation:
        SpaceInformation(shared_ptr[StateSpace] space)
        void setStateValidityChecker(const shared_ptr[StateValidityChecker] &svc)
        void setStateValidityCheckingResolution(double resolution)
        const shared_ptr[StateValidityChecker]& getStateValidityChecker() const
        void setup()

# cdef extern from "ompl/base/ScopedState.h" namespace "ompl::base":
#     cdef cppclass ScopedState[T]:
#         ScopedState(shared_ptr[SpaceInformation] si)
#         double& operator[](unsigned int i)

cdef extern from "ompl/base/ProblemDefinition.h" namespace "ompl::base":
    cdef cppclass ProblemDefinition:
        ProblemDefinition(shared_ptr[SpaceInformation] si)
        void setStartAndGoalStates(const State* start, const State* goal)
        libcpp.bool hasExactSolution()
        shared_ptr[Path] getSolutionPath()
        shared_ptr[OptimizationObjective] getOptimizationObjective()
        void setOptimizationObjective(const shared_ptr[OptimizationObjective] &objective)
        void clearSolutionPaths() const

cdef extern from "ompl/base/PlannerStatus.h" namespace "ompl::base":
    cdef cppclass PlannerStatus:
        pass

cdef extern from "ompl/base/Planner.h" namespace "ompl::base":
    cdef cppclass Planner:
        PlannerStatus solve(double solveTime)

cdef extern from "ompl/geometric/planners/rrt/RRTstar.h" namespace "ompl::geometric":
    cdef cppclass RRTstar(Planner):
        RRTstar(shared_ptr[SpaceInformation] si)
        void setProblemDefinition(shared_ptr[ProblemDefinition] pdef)
        void setup()
        void setRange(double distance)
        void clear()

cdef extern from "ompl/base/StateValidityChecker.h" namespace "ompl::base":
    cdef cppclass StateValidityChecker:
        libcpp.bool isValid(const State* state) const

cdef extern from "ompl/base/Cost.h" namespace "ompl::base":
    cdef cppclass Cost:
        Cost()
        Cost(double value)
        double value()

cdef extern from "ompl/base/Path.h" namespace "ompl::base":
    cdef cppclass Path:
        Cost cost(const shared_ptr[OptimizationObjective]& obj)

cdef extern from "ompl/geometric/PathGeometric.h" namespace "ompl::geometric":
    cdef cppclass PathGeometric:
        vector[State*] getStates()

cdef extern from "ompl/base/OptimizationObjective.h" namespace "ompl::base":
    cdef cppclass OptimizationObjective:
        pass
    cdef cppclass MultiOptimizationObjective(OptimizationObjective):
        MultiOptimizationObjective(const shared_ptr[SpaceInformation]& si)
        void addObjective(const shared_ptr[OptimizationObjective]& objective, double weight)

cdef extern from "ompl/base/objectives/StateCostIntegralObjective.h" namespace "ompl::base":
    cdef cppclass StateCostIntegralObjective(OptimizationObjective):
        Cost stateCost(const State* s) const

cdef extern from "ompl/base/objectives/PathLengthOptimizationObjective.h" namespace "ompl::base":
    cdef cppclass PathLengthOptimizationObjective(OptimizationObjective):
        PathLengthOptimizationObjective(const shared_ptr[SpaceInformation]& si)

# cdef extern from "ompl/base/spaces/RealVectorBounds.h" namespace "ompl::base":

