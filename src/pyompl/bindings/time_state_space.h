#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience alias
using TimeST = TimeStateSpace::StateType;

// =============================================================================
// WHAT IS TimeStateSpace?
// -----------------------
// TimeStateSpace represents TIME as a planning dimension.
// A state in this space is a single double: a moment in time.
//
// WHY PLAN OVER TIME?
//   In kinodynamic planning or spatio-temporal planning, the robot's
//   state includes WHEN it is at a location, not just WHERE.
//   Example: a robot that must reach a door exactly when it opens,
//   or a drone avoiding moving obstacles at specific times.
//
// TWO MODES:
//   1. UNBOUNDED (default after construction):
//      - setBounds() has NOT been called
//      - enforceBounds() is a no-op
//      - satisfiesBounds() always returns True
//      - sampleUniform() always produces time = 0
//      - getMaximumExtent() returns 1
//
//   2. BOUNDED (after calling setBounds(minTime, maxTime)):
//      - All operations behave as expected for a bounded interval
//      - sampleUniform() samples uniformly from [minTime, maxTime]
//      - enforceBounds() clamps to [minTime, maxTime]
//      - satisfiesBounds() checks if time is in [minTime, maxTime]
//
// DISTANCE: simply |t1 - t2| (absolute difference in time)
// INTERPOLATION: linear  result = from + t * (to - from)
//
// USE FOR:
//   - Kinodynamic planning with time constraints
//   - Spatio-temporal planning (avoid moving obstacles)
//   - Combined with other spaces via CompoundStateSpace
//
// C++ header: ompl/base/spaces/TimeStateSpace.h
// C++ class:  ompl::base::TimeStateSpace
// Inherits:   StateSpace
// =============================================================================

// -----------------------------------------------------------------------------
// TimeStateSpace::StateType helpers
// -----------------------------------------------------------------------------

static double timest_get_position(const TimeST &s)
{
    // C++: state->position
    // Returns the time value stored in this state.
    return s.position;
}

static void timest_set_position(TimeST &s, double v)
{
    // C++: state->position = v;
    // Sets the time value. Does NOT enforce bounds automatically.
    s.position = v;
}

static std::string timest_repr(const TimeST &s)
{
    return "<TimeState position=" + std::to_string(s.position) + ">";
}

// -----------------------------------------------------------------------------
// TimeStateSampler helpers
// -----------------------------------------------------------------------------

static void timesamp_sample_uniform(TimeStateSampler &s, TimeST *st)
{
    // C++: sampler->sampleUniform(state);
    // If BOUNDED: samples uniformly from [minTime, maxTime].
    // If UNBOUNDED: always sets position = 0.
    s.sampleUniform(static_cast<State *>(st));
}

static void timesamp_sample_uniform_near(TimeStateSampler &s, TimeST *st,
                                          const TimeST *near, double distance)
{
    // C++: sampler->sampleUniformNear(state, near, distance);
    // Samples uniformly from [near->position - distance, near->position + distance].
    // Clamped to bounds if the space is bounded.
    s.sampleUniformNear(static_cast<State *>(st),
                        static_cast<const State *>(near),
                        distance);
}

static void timesamp_sample_gaussian(TimeStateSampler &s, TimeST *st,
                                      const TimeST *mean, double stddev)
{
    // C++: sampler->sampleGaussian(state, mean, stdDev);
    // Samples from a Gaussian centred at mean->position with stdDev.
    // Clamped to bounds if the space is bounded.
    s.sampleGaussian(static_cast<State *>(st),
                     static_cast<const State *>(mean),
                     stddev);
}

// -----------------------------------------------------------------------------
// TimeStateSpace helpers
// -----------------------------------------------------------------------------

static void timess_set_bounds(TimeStateSpace &s, double minTime, double maxTime)
{
    // C++: space->setBounds(minTime, maxTime);
    // Sets the minimum and maximum time values.
    // Switches the space from UNBOUNDED to BOUNDED mode.
    // After this call:
    //   - isBounded() returns True
    //   - sampleUniform() samples from [minTime, maxTime]
    //   - enforceBounds() clamps to [minTime, maxTime]
    //   - satisfiesBounds() checks against [minTime, maxTime]
    //   - getMaximumExtent() returns maxTime - minTime
    s.setBounds(minTime, maxTime);
}

static double timess_get_min_time_bound(const TimeStateSpace &s)
{
    // C++: space->getMinTimeBound()
    // Returns the minimum time value. Returns 0 if unbounded.
    return s.getMinTimeBound();
}

static double timess_get_max_time_bound(const TimeStateSpace &s)
{
    // C++: space->getMaxTimeBound()
    // Returns the maximum time value. Returns 0 if unbounded.
    return s.getMaxTimeBound();
}

static bool timess_is_bounded(const TimeStateSpace &s)
{
    // C++: space->isBounded()
    // Returns True if setBounds() has been called, False otherwise.
    // This is the key flag that controls all behavior of the space.
    return s.isBounded();
}

static unsigned int timess_get_dimension(const TimeStateSpace &s)
{
    // C++: space->getDimension()
    // Always returns 1. Time is a 1-dimensional space.
    return s.getDimension();
}

static double timess_get_maximum_extent(const TimeStateSpace &s)
{
    // C++: space->getMaximumExtent()
    // If BOUNDED:   returns maxTime - minTime
    // If UNBOUNDED: returns 1  (a convention for unbounded spaces)
    return s.getMaximumExtent();
}

static double timess_get_measure(const TimeStateSpace &s)
{
    // C++: space->getMeasure()
    // If BOUNDED:   returns maxTime - minTime  (length of the interval)
    // If UNBOUNDED: returns 1
    return s.getMeasure();
}

static void timess_enforce_bounds(const TimeStateSpace &s, TimeST *st)
{
    // C++: space->enforceBounds(state);
    // If BOUNDED:   clamps state->position to [minTime, maxTime]
    // If UNBOUNDED: no-op (does nothing)
    s.enforceBounds(static_cast<State *>(st));
}

static bool timess_satisfies_bounds(const TimeStateSpace &s, const TimeST *st)
{
    // C++: space->satisfiesBounds(state)
    // If BOUNDED:   returns True if position is in [minTime, maxTime]
    // If UNBOUNDED: always returns True
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void timess_copy_state(const TimeStateSpace &s, TimeST *dst, const TimeST *src)
{
    // C++: space->copyState(destination, source);
    // Copies the time value from src to dst.
    s.copyState(static_cast<State *>(dst),
                static_cast<const State *>(src));
}

static TimeST *timess_clone_state(const TimeStateSpace &s, const TimeST *src)
{
    // C++: space->cloneState(src)->as<TimeStateSpace::StateType>()
    // Allocates a new state and copies src. Must be freed with freeState().
    return s.cloneState(static_cast<const State *>(src))->as<TimeST>();
}

static unsigned int timess_get_serialization_length(const TimeStateSpace &s)
{
    // C++: space->getSerializationLength()
    // Returns sizeof(double) = 8 bytes. Time is just one double.
    return s.getSerializationLength();
}

static double timess_distance(const TimeStateSpace &s,
                               const TimeST *s1, const TimeST *s2)
{
    // C++: space->distance(state1, state2)
    // Returns |s1->position - s2->position|
    // Simple absolute difference — time is a 1D Euclidean space.
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool timess_equal_states(const TimeStateSpace &s,
                                 const TimeST *s1, const TimeST *s2)
{
    // C++: space->equalStates(state1, state2)
    // Returns True if both states have exactly the same position value.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void timess_interpolate(const TimeStateSpace &s,
                                const TimeST *from, const TimeST *to,
                                double t, TimeST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // Linear interpolation: result->position = from + t * (to - from)
    // t=0 → from, t=1 → to, t=0.5 → midpoint in time.
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}

static TimeST *timess_alloc_state(TimeStateSpace &s)
{
    // C++: space->allocState()->as<TimeStateSpace::StateType>()
    // Allocates a new time state. Memory managed by the space.
    // State is uninitialised — set state.position before using.
    return s.allocState()->as<TimeST>();
}

static void timess_free_state(TimeStateSpace &s, TimeST *st)
{
    // C++: space->freeState(state);
    // Frees memory allocated by allocState().
    s.freeState(static_cast<State *>(st));
}

static StateSamplerPtr timess_alloc_default_sampler(const TimeStateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    // Returns a TimeStateSampler.
    // Behaviour depends on bounded/unbounded mode (see above).
    return s.allocDefaultStateSampler();
}

static StateSamplerPtr timess_alloc_state_sampler(const TimeStateSpace &s)
{
    // C++: space->allocStateSampler()
    return s.allocStateSampler();
}

static std::string timess_print_state(const TimeStateSpace &s, const TimeST *st)
{
    // C++: space->printState(state, std::cout);
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string timess_print_settings(const TimeStateSpace &s)
{
    // C++: space->printSettings(std::cout);
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void timess_register_projections(TimeStateSpace &s)
{
    // C++: space->registerProjections();
    // Called automatically by setup().
    s.registerProjections();
}

static void timess_setup(TimeStateSpace &s)
{
    // C++: space->setup();
    // Finalises the space.
    s.setup();
}

static std::vector<double> timess_copy_to_reals(const TimeStateSpace &s,
                                                  const TimeST *st)
{
    // C++: space->copyToReals(reals, source);
    // Returns [position] as a 1-element list.
    const_cast<TimeStateSpace &>(s).computeLocations();
    std::vector<double> reals;
    s.copyToReals(reals, static_cast<const State *>(st));
    return reals;
}

static void timess_copy_from_reals(const TimeStateSpace &s,
                                    TimeST *dst,
                                    const std::vector<double> &reals)
{
    // C++: space->copyFromReals(destination, reals);
    // Sets state from a 1-element list [position].
    const_cast<TimeStateSpace &>(s).computeLocations();
    s.copyFromReals(static_cast<State *>(dst), reals);
}

static std::string timess_repr(const TimeStateSpace &s)
{
    return "<TimeStateSpace bounded=" +
           std::string(s.isBounded() ? "True" : "False") +
           (s.isBounded() ? " [" + std::to_string(s.getMinTimeBound()) +
                            ", " + std::to_string(s.getMaxTimeBound()) + "]" : "") +
           ">";
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_time_state_space(py::module_ &m)
{
    // -------------------------------------------------------------------------
    // TimeStateSpace::StateType
    // -------------------------------------------------------------------------
    py::class_<TimeST>(m, "TimeState",
        "A single state in TimeStateSpace — one time value.\n\n"
        "C++ type:   ompl::base::TimeStateSpace::StateType\n"
        "C++ header: ompl/base/spaces/TimeStateSpace.h\n\n"
        "WHAT IS A TIME STATE?\n"
        "Just one double: state.position = the time value.\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocState() to get one.")

        .def_property("position",
            &timest_get_position,
            &timest_set_position,
            "The time value stored in this state.\n"
            "C++: state->position\n\n"
            "Reading:  t = state.position\n"
            "Writing:  state.position = 3.5\n\n"
            "Does NOT auto-enforce bounds. Call space.enforceBounds(state)\n"
            "if you set a value outside [minTime, maxTime].")

        .def("__repr__", &timest_repr);

    // -------------------------------------------------------------------------
    // TimeStateSampler
    // -------------------------------------------------------------------------
    py::class_<TimeStateSampler, StateSampler,
               std::shared_ptr<TimeStateSampler>>(m, "TimeStateSampler",
        "Sampler for TimeStateSpace.\n\n"
        "C++ class:  ompl::base::TimeStateSampler\n"
        "C++ header: ompl/base/spaces/TimeStateSpace.h\n\n"
        "Behaviour depends on whether the space is bounded or not.\n"
        "Do NOT construct directly. Use space.allocDefaultStateSampler().")

        .def("sampleUniform", &timesamp_sample_uniform, py::arg("state"),
            "Sample a time value uniformly.\n\n"
            "C++: sampler->sampleUniform(state);\n\n"
            "If BOUNDED:   samples uniformly from [minTime, maxTime]\n"
            "If UNBOUNDED: always sets position = 0")

        .def("sampleUniformNear", &timesamp_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample uniformly from [near.position - distance, near.position + distance].\n\n"
            "C++: sampler->sampleUniformNear(state, near, distance);\n\n"
            "Clamped to bounds if the space is bounded.")

        .def("sampleGaussian", &timesamp_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample from a Gaussian centred at mean.position.\n\n"
            "C++: sampler->sampleGaussian(state, mean, stdDev);\n\n"
            "Clamped to bounds if the space is bounded.");

    // -------------------------------------------------------------------------
    // TimeStateSpace
    // -------------------------------------------------------------------------
    py::class_<TimeStateSpace, StateSpace,
               std::shared_ptr<TimeStateSpace>>(m, "TimeStateSpace",
        "State space representing time — a single time value.\n\n"
        "C++ header: ompl/base/spaces/TimeStateSpace.h\n"
        "C++ class:  ompl::base::TimeStateSpace\n"
        "Inherits:   StateSpace\n\n"
        "TWO MODES:\n\n"
        "UNBOUNDED (default):\n"
        "  setBounds() not called\n"
        "  satisfiesBounds() always True\n"
        "  enforceBounds() is a no-op\n"
        "  sampleUniform() always gives position=0\n"
        "  getMaximumExtent() returns 1\n\n"
        "BOUNDED (after setBounds(minTime, maxTime)):\n"
        "  All operations work as expected\n"
        "  sampleUniform() samples from [minTime, maxTime]\n"
        "  enforceBounds() clamps to [minTime, maxTime]\n\n"
        "DISTANCE: |t1 - t2| (absolute difference)\n"
        "INTERPOLATION: linear\n\n"
        "USE FOR: kinodynamic planning, spatio-temporal planning,\n"
        "         any scenario where time is a planning dimension.")

        .def(py::init<>(),
            "Construct an unbounded TimeStateSpace.\n\n"
            "C++: auto space = std::make_shared<TimeStateSpace>();\n\n"
            "After construction the space is UNBOUNDED.\n"
            "Call setBounds(minTime, maxTime) to make it bounded.")

        .def("setBounds", &timess_set_bounds,
            py::arg("minTime"), py::arg("maxTime"),
            "Set the time bounds and switch to BOUNDED mode.\n\n"
            "C++: space->setBounds(minTime, maxTime);\n\n"
            "After this call:\n"
            "  - isBounded() returns True\n"
            "  - sampleUniform() samples from [minTime, maxTime]\n"
            "  - enforceBounds() clamps to [minTime, maxTime]\n"
            "  - satisfiesBounds() checks against [minTime, maxTime]\n"
            "  - getMaximumExtent() returns maxTime - minTime")

        .def("getMinTimeBound", &timess_get_min_time_bound,
            "Return the minimum time value.\n\n"
            "C++: space->getMinTimeBound()\n\n"
            "Returns 0 if the space is unbounded.")

        .def("getMaxTimeBound", &timess_get_max_time_bound,
            "Return the maximum time value.\n\n"
            "C++: space->getMaxTimeBound()\n\n"
            "Returns 0 if the space is unbounded.")

        .def("isBounded", &timess_is_bounded,
            "Return True if setBounds() has been called.\n\n"
            "C++: space->isBounded()\n\n"
            "This is the key flag controlling all space behaviour.\n"
            "Unbounded spaces have special no-op behaviour for bounds checking.")

        .def("getDimension", &timess_get_dimension,
            "Return the dimension: always 1.\n\n"
            "C++: space->getDimension()\n\n"
            "Time is a 1-dimensional space.")

        .def("getMaximumExtent", &timess_get_maximum_extent,
            "Return the maximum extent.\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "If BOUNDED:   returns maxTime - minTime\n"
            "If UNBOUNDED: returns 1")

        .def("getMeasure", &timess_get_measure,
            "Return the measure (length) of the time interval.\n\n"
            "C++: space->getMeasure()\n\n"
            "If BOUNDED:   returns maxTime - minTime\n"
            "If UNBOUNDED: returns 1")

        .def("satisfiesBounds", &timess_satisfies_bounds, py::arg("state"),
            "Return True if state.position is within [minTime, maxTime].\n\n"
            "C++: space->satisfiesBounds(state)\n\n"
            "If UNBOUNDED: always returns True.")

        .def("enforceBounds", &timess_enforce_bounds, py::arg("state"),
            "Clamp state.position to [minTime, maxTime].\n\n"
            "C++: space->enforceBounds(state);\n\n"
            "If UNBOUNDED: no-op (does nothing).")

        .def("copyState", &timess_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy the time value from source to destination.\n\n"
            "C++: space->copyState(destination, source);")

        .def("cloneState", &timess_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n\n"
            "C++: space->cloneState(src)->as<TimeStateSpace::StateType>()\n\n"
            "Must be freed with freeState().")

        .def("copyToReals", &timess_copy_to_reals, py::arg("source"),
            "Return [position] as a 1-element Python list.\n\n"
            "C++: space->copyToReals(reals, source);\n\n"
            "Requires setup() to have been called first.")

        .def("copyFromReals", &timess_copy_from_reals,
            py::arg("destination"), py::arg("reals"),
            "Set position from a 1-element Python list.\n\n"
            "C++: space->copyFromReals(destination, reals);\n\n"
            "Requires setup() to have been called first.")

        .def("getSerializationLength", &timess_get_serialization_length,
            "Return bytes to serialise one time state.\n\n"
            "C++: space->getSerializationLength()\n\n"
            "Returns 8 = sizeof(double).")

        .def("distance", &timess_distance, py::arg("state1"), py::arg("state2"),
            "Return the absolute time difference: |t1 - t2|.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "Time is a 1D Euclidean space — distance is just absolute difference.")

        .def("equalStates", &timess_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if both states have exactly the same time value.\n\n"
            "C++: space->equalStates(state1, state2)")

        .def("interpolate", &timess_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Linear interpolation between two time values.\n\n"
            "C++: space->interpolate(from, to, t, result);\n\n"
            "result.position = from.position + t * (to.position - from.position)\n"
            "t=0 → from,  t=1 → to,  t=0.5 → midpoint.")

        .def("allocState", &timess_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new time state. Memory managed by the space.\n\n"
            "C++: space->allocState()->as<TimeStateSpace::StateType>()\n\n"
            "State is uninitialised. Set state.position before using.\n"
            "Always call freeState() when done.")

        .def("freeState", &timess_free_state, py::arg("state"),
            "Free a state previously allocated by allocState().\n\n"
            "C++: space->freeState(state);")

        .def("allocDefaultStateSampler", &timess_alloc_default_sampler,
            "Allocate the default time sampler.\n\n"
            "C++: space->allocDefaultStateSampler()\n\n"
            "Returns a TimeStateSampler.")

        .def("allocStateSampler", &timess_alloc_state_sampler,
            "Allocate a sampler (custom if set, else default).\n\n"
            "C++: space->allocStateSampler()")

        .def("printState", &timess_print_state, py::arg("state"),
            "Return a string showing the time value.\n\n"
            "C++: space->printState(state, std::cout);")

        .def("printSettings", &timess_print_settings,
            "Return a string describing the space settings.\n\n"
            "C++: space->printSettings(std::cout);")

        .def("registerProjections", &timess_register_projections,
            "Register the default projection. Called automatically by setup().\n\n"
            "C++: space->registerProjections();")

        .def("setup", &timess_setup,
            "Finalise the space.\n\n"
            "C++: space->setup();")

        .def("getName", &TimeStateSpace::getName,
            "Return the name of this space.\n"
            "C++: space->getName()")

        .def("setName", &TimeStateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n"
            "C++: space->setName(name);")

        .def("__repr__", &timess_repr);
}