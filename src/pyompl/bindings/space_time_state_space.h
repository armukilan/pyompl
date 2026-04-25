#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/StateSampler.h>
#include <ompl/util/Exception.h>

#include <sstream>
#include <limits>

namespace py = pybind11;
using namespace ompl::base;

// =============================================================================
// WHAT IS SpaceTimeStateSpace?
// ----------------------------
// SpaceTimeStateSpace combines ANY state space with a TIME dimension.
// It is a CompoundStateSpace with:
//   Subspace 0: any StateSpace (position, orientation, etc.)
//   Subspace 1: TimeStateSpace
//
// WHAT MAKES IT SPECIAL?
// Unlike a generic CompoundStateSpace, SpaceTimeStateSpace has a
// VELOCITY-AWARE distance function:
//
//   The key concept: if a robot has maximum velocity vMax, it can only
//   travel a certain distance in a given time. The distance function
//   returns INFINITY if state1 cannot reach state2 in time (given vMax).
//
//   distance(s1, s2):
//     space_dist = euclidean distance in position space
//     time_dist  = |t2 - t1|
//     if space_dist / time_dist > vMax → return INFINITY (unreachable)
//     otherwise → return weighted compound distance
//
// WHY IS THIS USEFUL?
//   In kinodynamic planning (planning with dynamics), a robot cannot
//   teleport — it has a maximum speed. SpaceTimeStateSpace encodes this
//   physical constraint directly into the distance metric.
//
//   This is used by planners like SpaceTimeRRT to find trajectories
//   that respect velocity constraints.
//
// IMPORTANT NOTE — NOT A METRIC SPACE:
//   isMetricSpace() returns False because the triangle inequality
//   is not always satisfied (due to the infinity-returning distance).
//
// IMPORTANT NOTE — INFINITE MAXIMUM EXTENT:
//   getMaximumExtent() returns infinity because even with bounded time,
//   the distance can be infinite (if velocity constraint is violated).
//
// CONSTRUCTOR:
//   SpaceTimeStateSpace(spaceComponent, vMax=1.0, timeWeight=0.5)
//   spaceComponent — any StateSpacePtr (e.g. RealVectorStateSpace)
//   vMax           — maximum velocity of the space (units: space_unit/time_unit)
//   timeWeight     — weight of time in distance calculation
//
// C++ header: ompl/base/spaces/SpaceTimeStateSpace.h
// C++ class:  ompl::base::SpaceTimeStateSpace
// Inherits:   CompoundStateSpace → StateSpace
// =============================================================================

// Helper: get the state time from any state in a SpaceTimeStateSpace
static double stss_get_state_time(const State *st)
{
    // C++: SpaceTimeStateSpace::getStateTime(state)
    // Static helper that extracts the time component from any
    // SpaceTimeStateSpace state without needing a space reference.
    return SpaceTimeStateSpace::getStateTime(st);
}

// -----------------------------------------------------------------------------
// SpaceTimeStateSpace helpers
// NOTE: SpaceTimeStateSpace uses CompoundStateSpace::StateType as its state.
// The compound state has two substates:
//   substate[0] = state of the space component
//   substate[1] = TimeStateSpace::StateType (the time)
// We access them through CompoundState, which is what CompoundStateSpace stores.
// -----------------------------------------------------------------------------

static double stss_distance(const SpaceTimeStateSpace &s,
                             const State *s1, const State *s2)
{
    // C++: space->distance(state1, state2)
    // Returns the velocity-aware distance between two states.
    //
    // FORMULA:
    //   space_dist = distance in the position component
    //   time_diff  = |t2 - t1|
    //   if space_dist / time_diff > vMax → return INFINITY
    //   else → timeWeight * time_diff + (1-timeWeight) * space_dist
    //            (approximate — actual formula is in the C++ implementation)
    //
    // Returns float('inf') if the motion is physically impossible.
    return s.distance(s1, s2);
}

static double stss_time_to_cover_distance(const SpaceTimeStateSpace &s,
                                           const State *s1, const State *s2)
{
    // C++: space->timeToCoverDistance(state1, state2)
    // Returns the MINIMUM time needed to travel from state1 to state2
    // given the maximum velocity vMax.
    //
    // = distance_in_space / vMax
    //
    // This is the earliest arrival time at state2 if departing from state1.
    // Used internally by planners to check reachability.
    return s.timeToCoverDistance(s1, s2);
}

static double stss_distance_space(const SpaceTimeStateSpace &s,
                                   const State *s1, const State *s2)
{
    // C++: space->distanceSpace(state1, state2)
    // Returns ONLY the distance in the position (space) component.
    // Ignores the time component entirely.
    // Useful for checking how far apart two positions are
    // regardless of when the robot is at those positions.
    return s.distanceSpace(s1, s2);
}

static double stss_distance_time(const SpaceTimeStateSpace &s,
                                  const State *s1, const State *s2)
{
    // C++: space->distanceTime(state1, state2)
    // Returns ONLY the distance in the time component.
    // = |t1 - t2|
    // Ignores the position component entirely.
    return s.distanceTime(s1, s2);
}

static void stss_set_time_bounds(SpaceTimeStateSpace &s, double lb, double ub)
{
    // C++: space->setTimeBounds(lb, ub);
    // Sets the bounds on the time component of the state space.
    // Equivalent to calling getTimeComponent()->setBounds(lb, ub).
    // After this call the time sampler will sample from [lb, ub].
    s.setTimeBounds(lb, ub);
}

static double stss_get_v_max(const SpaceTimeStateSpace &s)
{
    // C++: space->getVMax()
    // Returns the maximum velocity of the space.
    // This is the maximum speed at which the robot can move
    // through the position space per unit time.
    // Used in the distance function to check feasibility.
    return s.getVMax();
}

static void stss_set_v_max(SpaceTimeStateSpace &s, double vMax)
{
    // C++: space->setVMax(vMax);
    // Changes the maximum velocity after construction.
    // A higher vMax means more states are reachable in a given time.
    // A lower vMax means the distance function returns infinity more often.
    s.setVMax(vMax);
}

static StateSpacePtr stss_get_space_component(SpaceTimeStateSpace &s)
{
    // C++: space->getSpaceComponent()
    // Returns the position (space) component as a StateSpacePtr.
    // This is subspace[0] of the compound space.
    return s.getSpaceComponent();
}

static TimeStateSpace *stss_get_time_component(SpaceTimeStateSpace &s)
{
    // C++: space->getTimeComponent()
    // Returns the time component as a raw TimeStateSpace pointer.
    // This is subspace[1] of the compound space.
    // Use this to set time bounds: getTimeComponent()->setBounds(0, 10)
    return s.getTimeComponent();
}

static bool stss_is_metric_space(const SpaceTimeStateSpace &s)
{
    // C++: space->isMetricSpace()
    // Always returns False for SpaceTimeStateSpace.
    // The distance function can return infinity, which violates
    // the triangle inequality required for a metric space.
    // Some planners require metric spaces — check compatibility.
    return s.isMetricSpace();
}

static double stss_get_maximum_extent(const SpaceTimeStateSpace &s)
{
    // C++: space->getMaximumExtent()
    // Always returns infinity for SpaceTimeStateSpace.
    // Even with bounded time, the distance can be infinite
    // (when velocity constraint is violated).
    return s.getMaximumExtent();
}

static void stss_update_epsilon(SpaceTimeStateSpace &s)
{
    // C++: space->updateEpsilon();
    // Recalculates the internal epsilon value after bounds change.
    // Call this after changing time bounds or space bounds.
    // epsilon is used for numerical precision in time distance calculation.
    s.updateEpsilon();
}

static void stss_setup(SpaceTimeStateSpace &s)
{
    // C++: space->setup();
    // Finalises the space.
    s.setup();
}

static std::string stss_repr(const SpaceTimeStateSpace &s)
{
    return "<SpaceTimeStateSpace vMax=" + std::to_string(s.getVMax()) + ">";
}

static State *stss_alloc_state(SpaceTimeStateSpace &s)
{
    // Returns raw State* — SpaceTimeStateSpace manages the compound memory.
    // Use only with space methods (distance, interpolate, sampler).
    return s.allocState();
}

static void stss_free_state(SpaceTimeStateSpace &s, State *st)
{
    s.freeState(st);
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_space_time_state_space(py::module_ &m)
{
    // Register ompl::Exception as a Python exception
    // pybind11 automatically catches std::runtime_error subclasses,
    // so ompl::Exception (which inherits std::runtime_error) will
    // be raised as RuntimeError in Python.

    // -------------------------------------------------------------------------
    // SpaceTimeStateSpace
    // -------------------------------------------------------------------------
    py::class_<SpaceTimeStateSpace, CompoundStateSpace,
               std::shared_ptr<SpaceTimeStateSpace>>(m, "SpaceTimeStateSpace",
        "A state space combining position + time with velocity-aware distance.\n\n"
        "C++ header: ompl/base/spaces/SpaceTimeStateSpace.h\n"
        "C++ class:  ompl::base::SpaceTimeStateSpace\n"
        "Inherits:   CompoundStateSpace → StateSpace\n\n"
        "WHAT IS SpaceTimeStateSpace?\n"
        "Combines any state space (position) with TimeStateSpace.\n"
        "The distance function is velocity-aware:\n"
        "  - If a robot cannot physically reach state2 from state1\n"
        "    given max velocity vMax, distance returns INFINITY.\n"
        "  - Otherwise returns a weighted distance.\n\n"
        "COMPOUND STRUCTURE:\n"
        "  Subspace 0: spaceComponent (any StateSpacePtr)\n"
        "  Subspace 1: TimeStateSpace\n\n"
        "KEY PROPERTIES:\n"
        "  isMetricSpace() = False  (triangle inequality not guaranteed)\n"
        "  getMaximumExtent() = infinity\n\n"
        "USE FOR: kinodynamic planning, spatio-temporal planning,\n"
        "         planning with velocity constraints.\n\n"
        "SETUP REQUIRED:\n"
        "  1. Create the space component\n"
        "  2. space = SpaceTimeStateSpace(spaceComponent, vMax, timeWeight)\n"
        "  3. space.setTimeBounds(minTime, maxTime)\n"
        "  4. space.setup()")

        .def(py::init<const StateSpacePtr &, double, double>(),
            py::arg("spaceComponent"),
            py::arg("vMax") = 1.0,
            py::arg("timeWeight") = 0.5,
            "Construct with a position space, max velocity, and time weight.\n\n"
            "C++: SpaceTimeStateSpace(spaceComponent, vMax, timeWeight)\n\n"
            "Parameters:\n"
            "  spaceComponent — any StateSpacePtr (e.g. RealVectorStateSpace)\n"
            "  vMax           — max velocity (space units / time unit). Default 1.0\n"
            "  timeWeight     — weight of time in distance. Default 0.5\n\n"
            "You MUST call setTimeBounds() to bound the time component.")

        .def("distance", &stss_distance,
            py::arg("state1"), py::arg("state2"),
            "Return velocity-aware distance between two states.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "Returns INFINITY if state2 is unreachable from state1\n"
            "given the maximum velocity vMax.\n\n"
            "Reachability check:\n"
            "  space_dist = distance in position component\n"
            "  time_diff  = |t2 - t1|\n"
            "  if space_dist / time_diff > vMax → return infinity\n\n"
            "Note: The distance is direction-independent for RRTConnect\n"
            "compatibility, even though physical motion has a time direction.\n"
            "Use motion validators to enforce forward-in-time constraint.")

        .def("timeToCoverDistance", &stss_time_to_cover_distance,
            py::arg("state1"), py::arg("state2"),
            "Return the minimum time needed to travel from state1 to state2.\n\n"
            "C++: space->timeToCoverDistance(state1, state2)\n\n"
            "= distanceSpace(state1, state2) / vMax\n\n"
            "This is the earliest arrival time at state2 given max velocity.\n"
            "If timeToCoverDistance > |t2 - t1|, the motion is infeasible.")

        .def("distanceSpace", &stss_distance_space,
            py::arg("state1"), py::arg("state2"),
            "Return distance in the position component only.\n\n"
            "C++: space->distanceSpace(state1, state2)\n\n"
            "Ignores time. Uses the space component's distance metric.\n"
            "Useful for checking how far apart positions are.")

        .def("distanceTime", &stss_distance_time,
            py::arg("state1"), py::arg("state2"),
            "Return distance in the time component only.\n\n"
            "C++: space->distanceTime(state1, state2)\n\n"
            "= |t1 - t2|\n"
            "Ignores position. Just the absolute time difference.")

        .def_static("getStateTime", &stss_get_state_time,
            py::arg("state"),
            "Extract the time value from any SpaceTimeStateSpace state.\n\n"
            "C++: SpaceTimeStateSpace::getStateTime(state)  (static method)\n\n"
            "This is a static method — call as SpaceTimeStateSpace.getStateTime(s).\n"
            "Returns the time component (substate[1].position) of the state.")

        .def("setTimeBounds", &stss_set_time_bounds,
            py::arg("lb"), py::arg("ub"),
            "Set the time bounds [lb, ub] for the time component.\n\n"
            "C++: space->setTimeBounds(lb, ub);\n\n"
            "MUST be called before using the space.\n"
            "Equivalent to: getTimeComponent()->setBounds(lb, ub)\n"
            "After this call the time sampler samples from [lb, ub].\n"
            "Call updateEpsilon() after changing bounds.")

        .def("getVMax", &stss_get_v_max,
            "Return the maximum velocity of the space.\n\n"
            "C++: space->getVMax()\n\n"
            "Units: space_units / time_unit\n"
            "Used in distance() to check if motions are physically feasible.")

        .def("setVMax", &stss_set_v_max, py::arg("vMax"),
            "Set the maximum velocity.\n\n"
            "C++: space->setVMax(vMax);\n\n"
            "Higher vMax → more states reachable → fewer infinity distances.\n"
            "Lower vMax  → fewer states reachable → more infinity distances.\n"
            "Call updateEpsilon() after changing vMax.")

        .def("getSpaceComponent", &stss_get_space_component,
            "Return the position (space) subspace as a StateSpacePtr.\n\n"
            "C++: space->getSpaceComponent()\n\n"
            "This is subspace[0] of the compound space.")

        .def("getTimeComponent", &stss_get_time_component,
            py::return_value_policy::reference_internal,
            "Return the TimeStateSpace component.\n\n"
            "C++: space->getTimeComponent()\n\n"
            "This is subspace[1] of the compound space.\n"
            "Use to access time bounds: getTimeComponent().getMinTimeBound()")

        .def("isMetricSpace", &stss_is_metric_space,
            "Return False — SpaceTimeStateSpace is NOT a metric space.\n\n"
            "C++: space->isMetricSpace()\n\n"
            "The distance function can return infinity, violating the\n"
            "triangle inequality. Some planners may not work with this space.")

        .def("getMaximumExtent", &stss_get_maximum_extent,
            "Return infinity — maximum extent is unbounded.\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "Even with bounded time, the distance can be infinite.")

        .def("updateEpsilon", &stss_update_epsilon,
            "Recalculate internal epsilon after bounds change.\n\n"
            "C++: space->updateEpsilon();\n\n"
            "Call after changing time bounds or vMax.\n"
            "epsilon is used for numerical precision in time calculations.")

        .def("setup", &stss_setup,
            "Finalise the space.\n\n"
            "C++: space->setup();")

        // .def("allocState", &stss_alloc_state,
        //     py::return_value_policy::reference_internal,
        //     "Allocate a new SpaceTimeStateSpace state.\n"
        //     "C++: space->allocState()\n"
        //     "Returns a raw State handle. Use only with space methods.")
        .def("allocState", &stss_alloc_state,
            py::return_value_policy::reference,
            "Allocate a new SpaceTimeStateSpace state.\n"
            "C++: space->allocState()\n"
            "Returns a raw State handle. Use only with space methods.")

        .def("freeState", &stss_free_state, py::arg("state"),
            "Free a previously allocated state.\n"
            "C++: space->freeState(state);")

        // Inherited CompoundStateSpace methods useful for accessing substates
        .def("getSubspaceCount",
            [](const SpaceTimeStateSpace &s) { return s.getSubspaceCount(); },
            "Return 2: subspace[0]=position, subspace[1]=time.\n\n"
            "C++: space->getSubspaceCount()")

        .def("getSubspaceWeight",
            [](const SpaceTimeStateSpace &s, unsigned int i) {
                return s.getSubspaceWeight(i);
            },
            py::arg("index"),
            "Return the weight of subspace i.\n\n"
            "C++: space->getSubspaceWeight(index)")

        .def("getDimension",
            [](const SpaceTimeStateSpace &s) { return s.getDimension(); },
            "Return the total dimension (space dim + 1 for time).\n\n"
            "C++: space->getDimension()")

        .def("getName", &SpaceTimeStateSpace::getName,
            "Return the name of this space.\n"
            "C++: space->getName()")

        .def("setName", &SpaceTimeStateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n"
            "C++: space->setName(name);")

        .def("__repr__", &stss_repr);


}