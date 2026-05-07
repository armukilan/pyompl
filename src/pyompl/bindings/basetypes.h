#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include <ompl/base/Cost.h>
#include <ompl/base/GoalTypes.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpaceTypes.h>

#include <sstream>
#include <string>

namespace py = pybind11;
using namespace ompl::base;

// =============================================================================
// WHAT IS THIS FILE?
// ------------------
// This file wraps four small "type definition" headers from ompl/base/.
// These types appear everywhere in OMPL — as return values, parameters,
// and enumerations used by planners, goals, spaces, and objectives.
//
// They are the FOUNDATION types. Everything else depends on them.
//
// Files wrapped:
//   ompl/base/Cost.h              → Cost class
//   ompl/base/GoalTypes.h         → GoalType enum
//   ompl/base/PlannerStatus.h     → PlannerStatus struct + StatusType enum
//   ompl/base/StateSpaceTypes.h   → StateSpaceType enum
// =============================================================================


// =============================================================================
// Cost helpers
// =============================================================================

static double cost_value(const Cost &c)
{
    // C++: c.value()
    // Returns the raw double value stored inside the Cost object.
    // Cost is just a type-safe wrapper around a double to prevent
    // accidentally mixing costs with other doubles (like distances).
    return c.value();
}

static bool cost_less_than(const Cost &a, const Cost &b)
{
    // No operator< in C++ Cost class — we expose it for convenience.
    // Used to compare: is cost A cheaper than cost B?
    return a.value() < b.value();
}

static bool cost_greater_than(const Cost &a, const Cost &b)
{
    return a.value() > b.value();
}

static bool cost_equal(const Cost &a, const Cost &b)
{
    return a.value() == b.value();
}

static Cost cost_add(const Cost &a, const Cost &b)
{
    // Combine two costs by addition. Used by combineCosts() in objectives.
    return Cost(a.value() + b.value());
}

static std::string cost_repr(const Cost &c)
{
    return "<Cost value=" + std::to_string(c.value()) + ">";
}

static std::string cost_str(const Cost &c)
{
    return std::to_string(c.value());
}


// =============================================================================
// PlannerStatus helpers
// =============================================================================

static std::string plannerstatus_as_string(const PlannerStatus &s)
{
    // C++: status.asString()
    // Returns a human-readable name for the status:
    //   "Exact solution", "Approximate solution", "Timeout", etc.
    return s.asString();
}

static bool plannerstatus_as_bool(const PlannerStatus &s)
{
    // C++: (bool)status  or  if (status) { ... }
    // Returns True if the planner found ANY solution (exact OR approximate).
    // Returns False for TIMEOUT, INVALID_START, CRASH, etc.
    // This is the most common check after solve():
    //   status = planner.solve(5.0)
    //   if status:  # found a solution
    return static_cast<bool>(s);
}

static PlannerStatus::StatusType plannerstatus_get_type(const PlannerStatus &s)
{
    // C++: (PlannerStatus::StatusType)status
    // Returns the raw enum value for detailed checking.
    return static_cast<PlannerStatus::StatusType>(s);
}

static std::string plannerstatus_repr(const PlannerStatus &s)
{
    return "<PlannerStatus " + s.asString() + ">";
}


// =============================================================================
// Binding function
// =============================================================================

inline void bind_base_types(py::module_ &m)
{
    // =========================================================================
    // Cost
    // C++ header: ompl/base/Cost.h
    // C++ class:  ompl::base::Cost
    // =========================================================================
    py::class_<Cost>(m, "Cost",
        "A type-safe wrapper around a double representing a path cost.\n\n"
        "C++ header: ompl/base/Cost.h\n"
        "C++ class:  ompl::base::Cost\n\n"
        "WHAT IS Cost?\n"
        "In OMPL, planning costs (path length, time, energy, etc.) are\n"
        "wrapped in this class instead of raw doubles. This prevents\n"
        "accidentally mixing cost values with other doubles like distances.\n\n"
        "Cost is used by:\n"
        "  - OptimizationObjective.motionCost(s1, s2) → Cost\n"
        "  - OptimizationObjective.stateCost(s) → Cost\n"
        "  - OptimizationObjective.combineCosts(c1, c2) → Cost\n"
        "  - ProblemDefinition.getSolutionPath().cost() → Cost\n\n"
        "EXAMPLES:\n"
        "  c1 = pyompl.Cost(3.14)     # create a cost\n"
        "  c2 = pyompl.Cost(2.71)\n"
        "  print(c1.value())           # 3.14\n"
        "  print(c1 < c2)              # False\n"
        "  print(c1 + c2)              # Cost(5.85)")

        .def(py::init<double>(), py::arg("v") = 0.0,
            "Construct a Cost with the given value.\n\n"
            "C++: Cost c(v);\n\n"
            "  c = pyompl.Cost(3.14)   # cost of 3.14\n"
            "  c = pyompl.Cost()       # cost of 0.0 (default)")

        .def("value", &cost_value,
            "Return the raw double value of this cost.\n\n"
            "C++: c.value()\n\n"
            "  c = pyompl.Cost(5.0)\n"
            "  v = c.value()   # v == 5.0\n\n"
            "This is the only getter — Cost stores exactly one double.")

        .def("__float__",  &cost_value,
            "Allow float(cost) conversion.\n\n"
            "  c = pyompl.Cost(3.14)\n"
            "  f = float(c)   # 3.14")

        .def("__lt__",     &cost_less_than,    py::arg("other"),
            "Return True if this cost is less than other.\n\n"
            "C++: a.value() < b.value()\n\n"
            "  Cost(2.0) < Cost(3.0)   # True")

        .def("__gt__",     &cost_greater_than, py::arg("other"),
            "Return True if this cost is greater than other.\n\n"
            "  Cost(3.0) > Cost(2.0)   # True")

        .def("__eq__",     &cost_equal,        py::arg("other"),
            "Return True if both costs have exactly the same value.\n\n"
            "  Cost(1.0) == Cost(1.0)  # True")

        .def("__add__",    &cost_add,          py::arg("other"),
            "Add two costs together, returning a new Cost.\n\n"
            "C++: Cost(a.value() + b.value())\n\n"
            "  c = Cost(2.0) + Cost(3.0)  # Cost(5.0)\n\n"
            "Used by OptimizationObjective.combineCosts().")

        .def("__repr__",   &cost_repr)
        .def("__str__",    &cost_str);


    // =========================================================================
    // GoalType enum
    // C++ header: ompl/base/GoalTypes.h
    // C++ enum:   ompl::base::GoalType
    // =========================================================================
    py::enum_<GoalType>(m, "GoalType",
        "Bit-flag enum identifying the type of a Goal object.\n\n"
        "C++ header: ompl/base/GoalTypes.h\n"
        "C++ enum:   ompl::base::GoalType\n\n"
        "WHAT IS GoalType?\n"
        "Every Goal subclass sets a type_ flag so planners can check\n"
        "what capabilities the goal has — can it be sampled? is it a\n"
        "single state? These flags allow safe downcasting.\n\n"
        "These are BIT FLAGS (can be OR'd together):\n"
        "  GOAL_ANY              — base goal (always set)\n"
        "  GOAL_REGION           — has isSatisfied(state) method\n"
        "  GOAL_SAMPLEABLE_REGION— can sample goal states\n"
        "  GOAL_STATE            — exactly one goal state\n"
        "  GOAL_STATES           — multiple goal states\n"
        "  GOAL_LAZY_SAMPLES     — lazily samples goal states\n\n"
        "USAGE:\n"
        "  goal = si.getGoal()  # once Goal.h is bound\n"
        "  if goal.getType() == pyompl.GoalType.GOAL_STATE:\n"
        "      ...  # it's a single-state goal")

        .value("GOAL_ANY",
               GoalType::GOAL_ANY,
               "Base goal type. Always set. Allows casting to Goal.\n"
               "C++: ompl::base::GOAL_ANY")

        .value("GOAL_REGION",
               GoalType::GOAL_REGION,
               "Goal has isSatisfied(state, distance) method.\n"
               "Allows casting to GoalRegion.\n"
               "C++: ompl::base::GOAL_REGION")

        .value("GOAL_SAMPLEABLE_REGION",
               GoalType::GOAL_SAMPLEABLE_REGION,
               "Goal can produce sample states via sampleGoal().\n"
               "Allows casting to GoalSampleableRegion.\n"
               "C++: ompl::base::GOAL_SAMPLEABLE_REGION")

        .value("GOAL_STATE",
               GoalType::GOAL_STATE,
               "Goal is exactly one state.\n"
               "Allows casting to GoalState.\n"
               "C++: ompl::base::GOAL_STATE")

        .value("GOAL_STATES",
               GoalType::GOAL_STATES,
               "Goal is a set of states.\n"
               "Allows casting to GoalStates.\n"
               "C++: ompl::base::GOAL_STATES")

        .value("GOAL_LAZY_SAMPLES",
               GoalType::GOAL_LAZY_SAMPLES,
               "Goal lazily generates samples (e.g. via IK solver).\n"
               "Allows casting to GoalLazySamples.\n"
               "C++: ompl::base::GOAL_LAZY_SAMPLES")

        .export_values();


    // =========================================================================
    // PlannerStatus::StatusType enum
    // C++ header: ompl/base/PlannerStatus.h
    // =========================================================================
    py::enum_<PlannerStatus::StatusType>(m, "PlannerStatusType",
        "The possible outcomes of a planner's solve() call.\n\n"
        "C++ header: ompl/base/PlannerStatus.h\n"
        "C++ enum:   ompl::base::PlannerStatus::StatusType\n\n"
        "USAGE:\n"
        "  status = planner.solve(5.0)\n"
        "  if status.getType() == pyompl.PlannerStatusType.EXACT_SOLUTION:\n"
        "      path = pdef.getSolutionPath()")

        .value("UNKNOWN",
               PlannerStatus::UNKNOWN,
               "Uninitialized. Planner has not been called yet.\n"
               "C++: PlannerStatus::UNKNOWN")

        .value("INVALID_START",
               PlannerStatus::INVALID_START,
               "Start state is invalid (in collision or outside bounds).\n"
               "Check your state validity checker and start state.\n"
               "C++: PlannerStatus::INVALID_START")

        .value("INVALID_GOAL",
               PlannerStatus::INVALID_GOAL,
               "Goal state is invalid or no goal was specified.\n"
               "C++: PlannerStatus::INVALID_GOAL")

        .value("UNRECOGNIZED_GOAL_TYPE",
               PlannerStatus::UNRECOGNIZED_GOAL_TYPE,
               "The planner does not support this type of goal.\n"
               "e.g., RRT needs GoalSampleableRegion but got GoalRegion.\n"
               "C++: PlannerStatus::UNRECOGNIZED_GOAL_TYPE")

        .value("TIMEOUT",
               PlannerStatus::TIMEOUT,
               "Time limit reached without finding any solution.\n"
               "Try increasing solve time or improving the problem setup.\n"
               "C++: PlannerStatus::TIMEOUT")

        .value("APPROXIMATE_SOLUTION",
               PlannerStatus::APPROXIMATE_SOLUTION,
               "Planner found a path that gets close to the goal\n"
               "but does not exactly reach it.\n"
               "bool(status) == True for this value.\n"
               "C++: PlannerStatus::APPROXIMATE_SOLUTION")

        .value("EXACT_SOLUTION",
               PlannerStatus::EXACT_SOLUTION,
               "Planner found a path that exactly reaches the goal.\n"
               "bool(status) == True for this value.\n"
               "C++: PlannerStatus::EXACT_SOLUTION")

        .value("CRASH",
               PlannerStatus::CRASH,
               "The planner crashed (unexpected error).\n"
               "C++: PlannerStatus::CRASH")

        .value("ABORT",
               PlannerStatus::ABORT,
               "Planning was aborted for some other reason.\n"
               "C++: PlannerStatus::ABORT")

        .value("INFEASIBLE",
               PlannerStatus::INFEASIBLE,
               "Planner determined the problem has no solution.\n"
               "C++: PlannerStatus::INFEASIBLE")

        .value("TYPE_COUNT",
               PlannerStatus::TYPE_COUNT,
               "Number of status types. Not a real status value.\n"
               "C++: PlannerStatus::TYPE_COUNT")

        .export_values();


    // =========================================================================
    // PlannerStatus struct
    // C++ header: ompl/base/PlannerStatus.h
    // C++ class:  ompl::base::PlannerStatus
    // =========================================================================
    py::class_<PlannerStatus>(m, "PlannerStatus",
        "The result returned by Planner::solve().\n\n"
        "C++ header: ompl/base/PlannerStatus.h\n"
        "C++ class:  ompl::base::PlannerStatus\n\n"
        "WHAT IS PlannerStatus?\n"
        "After calling planner.solve(time), you get a PlannerStatus.\n"
        "It tells you WHY the planner stopped and WHETHER a solution\n"
        "was found.\n\n"
        "THE MOST IMPORTANT PATTERN:\n"
        "  status = planner.solve(5.0)\n"
        "  if status:               # True = found a solution (exact or approx)\n"
        "      path = pdef.getSolutionPath()\n\n"
        "DETAILED CHECK:\n"
        "  if status.getType() == pyompl.PlannerStatusType.EXACT_SOLUTION:\n"
        "      print('Found exact solution!')\n"
        "  elif status.getType() == pyompl.PlannerStatusType.APPROXIMATE_SOLUTION:\n"
        "      print('Only approximate solution found')\n"
        "  else:\n"
        "      print('Failed:', status.asString())")

        .def(py::init<PlannerStatus::StatusType>(),
            py::arg("status") = PlannerStatus::UNKNOWN,
            "Construct with a specific StatusType.\n\n"
            "C++: PlannerStatus status(PlannerStatus::EXACT_SOLUTION);\n\n"
            "  s = pyompl.PlannerStatus(pyompl.PlannerStatusType.EXACT_SOLUTION)")

        .def(py::init<bool, bool>(),
            py::arg("hasSolution"), py::arg("isApproximate"),
            "Construct from booleans: found solution? approximate?\n\n"
            "C++: PlannerStatus status(hasSolution, isApproximate);\n\n"
            "  s = pyompl.PlannerStatus(True, False)   # EXACT_SOLUTION\n"
            "  s = pyompl.PlannerStatus(True, True)    # APPROXIMATE_SOLUTION\n"
            "  s = pyompl.PlannerStatus(False, False)  # TIMEOUT")

        .def("asString", &plannerstatus_as_string,
            "Return a human-readable string for this status.\n\n"
            "C++: status.asString()\n\n"
            "Returns one of:\n"
            "  'Unknown', 'Invalid start', 'Invalid goal',\n"
            "  'Unrecognized goal type', 'Timeout',\n"
            "  'Approximate solution', 'Exact solution',\n"
            "  'Crash', 'Abort', 'Infeasible'\n\n"
            "  s = pyompl.PlannerStatus(pyompl.PlannerStatusType.EXACT_SOLUTION)\n"
            "  print(s.asString())   # 'Exact solution'")

        .def("getType", &plannerstatus_get_type,
            "Return the raw StatusType enum value.\n\n"
            "C++: (PlannerStatus::StatusType)status\n\n"
            "Use for detailed comparisons:\n"
            "  t = status.getType()\n"
            "  if t == pyompl.PlannerStatusType.EXACT_SOLUTION: ...")

        .def("__bool__", &plannerstatus_as_bool,
            "Return True if an approximate or exact solution was found.\n\n"
            "C++: (bool)status  or  if (status) { ... }\n\n"
            "This is the most common check:\n"
            "  if status:   # True for EXACT or APPROXIMATE solution\n"
            "      path = pdef.getSolutionPath()\n\n"
            "Returns False for: UNKNOWN, INVALID_START, INVALID_GOAL,\n"
            "UNRECOGNIZED_GOAL_TYPE, TIMEOUT, CRASH, ABORT, INFEASIBLE")

        .def("__repr__", &plannerstatus_repr)

        .def("__str__",  &plannerstatus_as_string);


    // =========================================================================
    // StateSpaceType enum
    // C++ header: ompl/base/StateSpaceTypes.h
    // C++ enum:   ompl::base::StateSpaceType
    // =========================================================================
    py::enum_<StateSpaceType>(m, "StateSpaceType",
        "Integer type identifier for each OMPL state space.\n\n"
        "C++ header: ompl/base/StateSpaceTypes.h\n"
        "C++ enum:   ompl::base::StateSpaceType\n\n"
        "WHAT IS StateSpaceType?\n"
        "Every state space sets its type_ field in the constructor.\n"
        "You can call space.getType() to check which kind of space it is.\n\n"
        "USAGE:\n"
        "  space = pyompl.RealVectorStateSpace(3)\n"
        "  if space.getType() == pyompl.StateSpaceType.STATE_SPACE_REAL_VECTOR:\n"
        "      print('It is R^n')\n\n"
        "This is mainly used internally by OMPL for safe downcasting.\n"
        "In Python you can also use isinstance() instead.")

        .value("STATE_SPACE_UNKNOWN",
               StateSpaceType::STATE_SPACE_UNKNOWN,
               "Unset type (default).\n"
               "C++: ompl::base::STATE_SPACE_UNKNOWN")

        .value("STATE_SPACE_REAL_VECTOR",
               StateSpaceType::STATE_SPACE_REAL_VECTOR,
               "RealVectorStateSpace — R^n.\n"
               "C++: ompl::base::STATE_SPACE_REAL_VECTOR")

        .value("STATE_SPACE_SO2",
               StateSpaceType::STATE_SPACE_SO2,
               "SO2StateSpace — 2D rotations (circle).\n"
               "C++: ompl::base::STATE_SPACE_SO2")

        .value("STATE_SPACE_SO3",
               StateSpaceType::STATE_SPACE_SO3,
               "SO3StateSpace — 3D rotations (quaternion).\n"
               "C++: ompl::base::STATE_SPACE_SO3")

        .value("STATE_SPACE_SE2",
               StateSpaceType::STATE_SPACE_SE2,
               "SE2StateSpace — 2D position + heading.\n"
               "C++: ompl::base::STATE_SPACE_SE2")

        .value("STATE_SPACE_SE3",
               StateSpaceType::STATE_SPACE_SE3,
               "SE3StateSpace — 3D position + orientation.\n"
               "C++: ompl::base::STATE_SPACE_SE3")

        .value("STATE_SPACE_TIME",
               StateSpaceType::STATE_SPACE_TIME,
               "TimeStateSpace — a time value.\n"
               "C++: ompl::base::STATE_SPACE_TIME")

        .value("STATE_SPACE_DISCRETE",
               StateSpaceType::STATE_SPACE_DISCRETE,
               "DiscreteStateSpace — integer states.\n"
               "C++: ompl::base::STATE_SPACE_DISCRETE")

        .value("STATE_SPACE_DUBINS",
               StateSpaceType::STATE_SPACE_DUBINS,
               "DubinsStateSpace — car-like Dubins curves.\n"
               "C++: ompl::base::STATE_SPACE_DUBINS")

        .value("STATE_SPACE_REEDS_SHEPP",
               StateSpaceType::STATE_SPACE_REEDS_SHEPP,
               "ReedsSheppStateSpace — car with reverse.\n"
               "C++: ompl::base::STATE_SPACE_REEDS_SHEPP")

        .value("STATE_SPACE_MOBIUS",
               StateSpaceType::STATE_SPACE_MOBIUS,
               "MobiusStateSpace — Möbius strip manifold.\n"
               "C++: ompl::base::STATE_SPACE_MOBIUS")

        .value("STATE_SPACE_SPHERE",
               StateSpaceType::STATE_SPACE_SPHERE,
               "SphereStateSpace — S^2 manifold.\n"
               "C++: ompl::base::STATE_SPACE_SPHERE")

        .value("STATE_SPACE_TORUS",
               StateSpaceType::STATE_SPACE_TORUS,
               "TorusStateSpace — torus manifold.\n"
               "C++: ompl::base::STATE_SPACE_TORUS")

        .value("STATE_SPACE_KLEIN_BOTTLE",
               StateSpaceType::STATE_SPACE_KLEIN_BOTTLE,
               "KleinBottleStateSpace.\n"
               "C++: ompl::base::STATE_SPACE_KLEIN_BOTTLE")

        .value("STATE_SPACE_VANA",
               StateSpaceType::STATE_SPACE_VANA,
               "VanaStateSpace — 3D Dubins variant.\n"
               "C++: ompl::base::STATE_SPACE_VANA")

        .value("STATE_SPACE_OWEN",
               StateSpaceType::STATE_SPACE_OWEN,
               "OwenStateSpace — 3D Dubins fixed-wing aircraft.\n"
               "C++: ompl::base::STATE_SPACE_OWEN")

        .value("STATE_SPACE_VANA_OWEN",
               StateSpaceType::STATE_SPACE_VANA_OWEN,
               "VanaOwenStateSpace — combined Vana+Owen.\n"
               "C++: ompl::base::STATE_SPACE_VANA_OWEN")

        .value("STATE_SPACE_TROCHOID",
               StateSpaceType::STATE_SPACE_TROCHOID,
               "TrochoidStateSpace — marine vehicle motion.\n"
               "C++: ompl::base::STATE_SPACE_TROCHOID")

        .value("STATE_SPACE_TYPE_COUNT",
               StateSpaceType::STATE_SPACE_TYPE_COUNT,
               "Total number of built-in types. Not a real space.\n"
               "C++: ompl::base::STATE_SPACE_TYPE_COUNT")

        .export_values();
}