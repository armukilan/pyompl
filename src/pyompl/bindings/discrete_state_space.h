#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience alias
using DiscST = DiscreteStateSpace::StateType;

// =============================================================================
// WHAT IS DiscreteStateSpace?
// ---------------------------
// DiscreteStateSpace represents a FINITE SET of integer states.
// A state is a single integer in the range [lowerBound, upperBound] inclusive.
//
// UNLIKE all other state spaces we have seen:
//   - States are INTEGERS, not doubles
//   - There is NO continuous topology — you jump from one state to another
//   - Distance is simply |value1 - value2| (integer difference)
//   - Interpolation rounds to the nearest integer
//   - isDiscrete() returns True
//   - NO wrapping — lowerBound and upperBound are hard walls
//
// EXAMPLES OF USE:
//   - Robot gear selection: {1, 2, 3, 4, 5}
//   - Traffic light phase: {RED=0, YELLOW=1, GREEN=2}
//   - Elevator floor: {0, 1, 2, ..., N}
//   - Mode switching: {IDLE=0, MOVING=1, GRASPING=2, PLACING=3}
//   - Number of fingers gripping an object: {0, 1, 2, 3, 4, 5}
//
// COMBINED WITH OTHER SPACES:
//   DiscreteStateSpace is almost always used as part of a CompoundStateSpace
//   combined with continuous spaces. For example:
//     - (x, y, gear) = RealVectorStateSpace(2) + DiscreteStateSpace(1, 5)
//     - (x, y, mode) = RealVectorStateSpace(2) + DiscreteStateSpace(0, 3)
//
// CONSTRUCTOR:
//   DiscreteStateSpace(lowerBound, upperBound)
//   Both bounds are INCLUSIVE. The state count = upperBound - lowerBound + 1.
//
// C++ header: ompl/base/spaces/DiscreteStateSpace.h
// C++ class:  ompl::base::DiscreteStateSpace
// Inherits:   StateSpace
// =============================================================================

// -----------------------------------------------------------------------------
// DiscreteStateSpace::StateType helpers
// -----------------------------------------------------------------------------

static int discst_get_value(const DiscST &s)
{
    // C++: state->value
    // Returns the integer state value.
    return s.value;
}

static void discst_set_value(DiscST &s, int v)
{
    // C++: state->value = v;
    // Sets the integer state value.
    // Does NOT check bounds — call satisfiesBounds() or enforceBounds() separately.
    s.value = v;
}

static std::string discst_repr(const DiscST &s)
{
    return "<DiscreteState value=" + std::to_string(s.value) + ">";
}

// -----------------------------------------------------------------------------
// DiscreteStateSampler helpers
// -----------------------------------------------------------------------------

static void discsamp_sample_uniform(DiscreteStateSampler &s, DiscST *st)
{
    // C++: sampler->sampleUniform(state);
    // Samples an integer uniformly at random from [lowerBound, upperBound].
    // Every integer in the range is equally likely.
    s.sampleUniform(static_cast<State *>(st));
}

static void discsamp_sample_uniform_near(DiscreteStateSampler &s, DiscST *st,
                                          const DiscST *near, double distance)
{
    // C++: sampler->sampleUniformNear(state, near, distance);
    // Samples uniformly from integers within 'distance' of near->value.
    // Range: [near->value - floor(distance), near->value + floor(distance)]
    // Clamped to [lowerBound, upperBound].
    s.sampleUniformNear(static_cast<State *>(st),
                        static_cast<const State *>(near),
                        distance);
}

static void discsamp_sample_gaussian(DiscreteStateSampler &s, DiscST *st,
                                      const DiscST *mean, double stddev)
{
    // C++: sampler->sampleGaussian(state, mean, stdDev);
    // Samples from a discrete Gaussian (rounded normal distribution)
    // centred at mean->value with standard deviation stdDev.
    // Result is clamped to [lowerBound, upperBound].
    s.sampleGaussian(static_cast<State *>(st),
                     static_cast<const State *>(mean),
                     stddev);
}

// -----------------------------------------------------------------------------
// DiscreteStateSpace helpers
// -----------------------------------------------------------------------------

static int discss_get_lower_bound(const DiscreteStateSpace &s)
{
    // C++: space->getLowerBound()
    // Returns the lowest valid integer state.
    return s.getLowerBound();
}

static int discss_get_upper_bound(const DiscreteStateSpace &s)
{
    // C++: space->getUpperBound()
    // Returns the highest valid integer state.
    return s.getUpperBound();
}

static unsigned int discss_get_state_count(const DiscreteStateSpace &s)
{
    // C++: space->getStateCount()
    // Returns upperBound - lowerBound + 1.
    // This is the total number of distinct states in this space.
    return s.getStateCount();
}

static void discss_set_bounds(DiscreteStateSpace &s, int lower, int upper)
{
    // C++: space->setBounds(lowerBound, upperBound);
    // Changes the bounds of the discrete space after construction.
    // Both bounds are inclusive.
    // Call setup() after changing bounds if the space is already in use.
    s.setBounds(lower, upper);
}

static bool discss_is_discrete(const DiscreteStateSpace &s)
{
    // C++: space->isDiscrete()
    // Always returns True for DiscreteStateSpace.
    // This distinguishes it from continuous spaces.
    return s.isDiscrete();
}

static unsigned int discss_get_dimension(const DiscreteStateSpace &s)
{
    // C++: space->getDimension()
    // Returns 1. A discrete space is 1-dimensional (one integer value).
    return s.getDimension();
}

static double discss_get_maximum_extent(const DiscreteStateSpace &s)
{
    // C++: space->getMaximumExtent()
    // Returns upperBound - lowerBound.
    // This is the maximum possible distance between any two states.
    return s.getMaximumExtent();
}

static double discss_get_measure(const DiscreteStateSpace &s)
{
    // C++: space->getMeasure()
    // Returns the number of states: upperBound - lowerBound + 1.
    // This is the "volume" (count) of the discrete space.
    return s.getMeasure();
}

static void discss_enforce_bounds(const DiscreteStateSpace &s, DiscST *st)
{
    // C++: space->enforceBounds(state);
    // Clamps state->value to [lowerBound, upperBound].
    // If value < lowerBound → value = lowerBound
    // If value > upperBound → value = upperBound
    s.enforceBounds(static_cast<State *>(st));
}

static bool discss_satisfies_bounds(const DiscreteStateSpace &s, const DiscST *st)
{
    // C++: space->satisfiesBounds(state)
    // Returns True if lowerBound <= state->value <= upperBound.
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void discss_copy_state(const DiscreteStateSpace &s, DiscST *dst, const DiscST *src)
{
    // C++: space->copyState(destination, source);
    // Copies the integer value from src to dst.
    s.copyState(static_cast<State *>(dst),
                static_cast<const State *>(src));
}

static DiscST *discss_clone_state(const DiscreteStateSpace &s, const DiscST *src)
{
    // C++: space->cloneState(src)->as<DiscreteStateSpace::StateType>()
    // Allocates a new state and copies src. Must be freed with freeState().
    return s.cloneState(static_cast<const State *>(src))->as<DiscST>();
}

static unsigned int discss_get_serialization_length(const DiscreteStateSpace &s)
{
    // C++: space->getSerializationLength()
    // Returns sizeof(int) = 4 bytes.
    // The state is just one integer.
    return s.getSerializationLength();
}

static double discss_distance(const DiscreteStateSpace &s,
                               const DiscST *s1, const DiscST *s2)
{
    // C++: space->distance(state1, state2)
    // Returns |s1->value - s2->value| as a double.
    // Distance between adjacent integers is 1.
    // Distance is NOT metric in the mathematical sense because
    // interpolation is rounded, but it is symmetric and non-negative.
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool discss_equal_states(const DiscreteStateSpace &s,
                                 const DiscST *s1, const DiscST *s2)
{
    // C++: space->equalStates(state1, state2)
    // Returns True if both states have exactly the same integer value.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void discss_interpolate(const DiscreteStateSpace &s,
                                const DiscST *from, const DiscST *to,
                                double t, DiscST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // Computes: result->value = round(from->value + t * (to->value - from->value))
    // t=0 → from, t=1 → to, t=0.5 → rounded midpoint.
    // Because states are integers, interpolation ROUNDS to the nearest integer.
    // This means intermediate states are valid discrete states.
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}

static DiscST *discss_alloc_state(DiscreteStateSpace &s)
{
    // C++: space->allocState()->as<DiscreteStateSpace::StateType>()
    // Allocates a new discrete state. Memory managed by the space.
    // The state is uninitialised — set state.value before using.
    // Always call freeState() when done.
    return s.allocState()->as<DiscST>();
}

static void discss_free_state(DiscreteStateSpace &s, DiscST *st)
{
    // C++: space->freeState(state);
    // Frees memory allocated by allocState().
    s.freeState(static_cast<State *>(st));
}

static StateSamplerPtr discss_alloc_default_sampler(const DiscreteStateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    // Returns a DiscreteStateSampler for this space.
    return s.allocDefaultStateSampler();
}

static StateSamplerPtr discss_alloc_state_sampler(const DiscreteStateSpace &s)
{
    // C++: space->allocStateSampler()
    return s.allocStateSampler();
}

static std::string discss_print_state(const DiscreteStateSpace &s, const DiscST *st)
{
    // C++: space->printState(state, std::cout);
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string discss_print_settings(const DiscreteStateSpace &s)
{
    // C++: space->printSettings(std::cout);
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void discss_register_projections(DiscreteStateSpace &s)
{
    // C++: space->registerProjections();
    // Called automatically by setup().
    s.registerProjections();
}

static void discss_setup(DiscreteStateSpace &s)
{
    // C++: space->setup();
    s.setup();
}

// Direct sampling helpers (avoids base-class cast issues)
static void discss_sample_uniform(DiscreteStateSpace &s, DiscST *st)
{
    // C++: space->allocDefaultStateSampler()->sampleUniform(state);
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniform(static_cast<State *>(st));
}

static void discss_sample_uniform_near(DiscreteStateSpace &s, DiscST *st,
                                        const DiscST *near, double distance)
{
    // C++: space->allocDefaultStateSampler()->sampleUniformNear(state, near, distance);
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniformNear(static_cast<State *>(st),
                               static_cast<const State *>(near), distance);
}

static void discss_sample_gaussian(DiscreteStateSpace &s, DiscST *st,
                                    const DiscST *mean, double stddev)
{
    // C++: space->allocDefaultStateSampler()->sampleGaussian(state, mean, stdDev);
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleGaussian(static_cast<State *>(st),
                            static_cast<const State *>(mean), stddev);
}

static std::string discss_repr(const DiscreteStateSpace &s)
{
    return "<DiscreteStateSpace [" + std::to_string(s.getLowerBound()) +
           ", " + std::to_string(s.getUpperBound()) +
           "] count=" + std::to_string(s.getStateCount()) + ">";
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_discrete_state_space(py::module_ &m)
{
    // -------------------------------------------------------------------------
    // DiscreteStateSpace::StateType
    // -------------------------------------------------------------------------
    py::class_<DiscST>(m, "DiscreteState",
        "A single state in DiscreteStateSpace — one integer value.\n\n"
        "C++ type:   ompl::base::DiscreteStateSpace::StateType\n"
        "C++ header: ompl/base/spaces/DiscreteStateSpace.h\n\n"
        "WHAT IS A DISCRETE STATE?\n"
        "Just one integer: state.value ∈ [lowerBound, upperBound].\n\n"
        "Examples:\n"
        "  state.value = 0   ← gear 1, floor 0, IDLE mode\n"
        "  state.value = 3   ← gear 4, floor 3, GRASPING mode\n\n"
        "Do NOT construct directly. Use space.allocState().")

        .def_property("value",
            &discst_get_value,
            &discst_set_value,
            "The integer state value. Must be in [lowerBound, upperBound].\n"
            "C++: state->value\n\n"
            "Reading:  v = state.value\n"
            "Writing:  state.value = 3\n\n"
            "Does NOT auto-enforce bounds. Call space.enforceBounds(state)\n"
            "if you set a value outside [lowerBound, upperBound].")

        .def("__repr__", &discst_repr);

    // -------------------------------------------------------------------------
    // DiscreteStateSampler
    // -------------------------------------------------------------------------
    py::class_<DiscreteStateSampler, StateSampler,
               std::shared_ptr<DiscreteStateSampler>>(m, "DiscreteStateSampler",
        "Sampler for DiscreteStateSpace — generates random integers.\n\n"
        "C++ class:  ompl::base::DiscreteStateSampler\n"
        "C++ header: ompl/base/spaces/DiscreteStateSpace.h\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocDefaultStateSampler() to get one.\n"
        "Or use space.sampleUniform() etc. directly.")

        .def("sampleUniform", &discsamp_sample_uniform, py::arg("state"),
            "Sample a uniformly random integer from [lowerBound, upperBound].\n\n"
            "C++: sampler->sampleUniform(state);\n\n"
            "Every integer in the range is equally likely.")

        .def("sampleUniformNear", &discsamp_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample uniformly from integers within 'distance' of 'near'.\n\n"
            "C++: sampler->sampleUniformNear(state, near, distance);\n\n"
            "Range: [near.value - floor(distance), near.value + floor(distance)]\n"
            "Clamped to [lowerBound, upperBound].")

        .def("sampleGaussian", &discsamp_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample from a discrete Gaussian centred at 'mean'.\n\n"
            "C++: sampler->sampleGaussian(state, mean, stdDev);\n\n"
            "Result rounded to nearest integer and clamped to bounds.");

    // -------------------------------------------------------------------------
    // DiscreteStateSpace
    // -------------------------------------------------------------------------
    py::class_<DiscreteStateSpace, StateSpace,
               std::shared_ptr<DiscreteStateSpace>>(m, "DiscreteStateSpace",
        "State space of discrete integer states in [lowerBound, upperBound].\n\n"
        "C++ header: ompl/base/spaces/DiscreteStateSpace.h\n"
        "C++ class:  ompl::base::DiscreteStateSpace\n"
        "Inherits:   StateSpace\n\n"
        "WHAT IS A DISCRETE STATE SPACE?\n"
        "A finite set of integer states. No continuous topology.\n"
        "Distance = |value1 - value2|  (integer steps)\n"
        "Interpolation = rounded linear interpolation\n"
        "isDiscrete() = True\n\n"
        "BOUNDS: Both lowerBound and upperBound are INCLUSIVE.\n"
        "Total states = upperBound - lowerBound + 1\n"
        "NO wrapping — states do not loop around.\n\n"
        "USE FOR: gear selection, mode switching, floor number,\n"
        "         any finite set of integer configurations.")

        .def(py::init<int, int>(),
            py::arg("lowerBound"), py::arg("upperBound"),
            "Construct with inclusive integer bounds.\n\n"
            "C++: DiscreteStateSpace space(lowerBound, upperBound);\n\n"
            "Example:\n"
            "  space = DiscreteStateSpace(0, 5)   # states: 0,1,2,3,4,5\n"
            "  space = DiscreteStateSpace(1, 4)   # states: 1,2,3,4\n"
            "  space = DiscreteStateSpace(-2, 2)  # states: -2,-1,0,1,2")

        .def("getLowerBound", &discss_get_lower_bound,
            "Return the lowest valid integer state.\n\n"
            "C++: space->getLowerBound()")

        .def("getUpperBound", &discss_get_upper_bound,
            "Return the highest valid integer state.\n\n"
            "C++: space->getUpperBound()")

        .def("getStateCount", &discss_get_state_count,
            "Return the total number of distinct states.\n\n"
            "C++: space->getStateCount()\n\n"
            "= upperBound - lowerBound + 1")

        .def("setBounds", &discss_set_bounds,
            py::arg("lowerBound"), py::arg("upperBound"),
            "Change the bounds after construction.\n\n"
            "C++: space->setBounds(lowerBound, upperBound);\n\n"
            "Both bounds are inclusive.\n"
            "Call setup() after changing bounds if space is already in use.")

        .def("isDiscrete", &discss_is_discrete,
            "Return True — this is always a discrete space.\n\n"
            "C++: space->isDiscrete()")

        .def("getDimension", &discss_get_dimension,
            "Return the dimension: always 1.\n\n"
            "C++: space->getDimension()\n\n"
            "Discrete spaces are 1-dimensional (one integer value).")

        .def("getMaximumExtent", &discss_get_maximum_extent,
            "Return the maximum distance between any two states.\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "= upperBound - lowerBound\n"
            "Achieved between the two extreme states.")

        .def("getMeasure", &discss_get_measure,
            "Return the number of states (measure of discrete space).\n\n"
            "C++: space->getMeasure()\n\n"
            "= upperBound - lowerBound + 1\n"
            "Same as getStateCount() but as a double.")

        .def("satisfiesBounds", &discss_satisfies_bounds, py::arg("state"),
            "Return True if lowerBound <= state.value <= upperBound.\n\n"
            "C++: space->satisfiesBounds(state)")

        .def("enforceBounds", &discss_enforce_bounds, py::arg("state"),
            "Clamp state.value to [lowerBound, upperBound].\n\n"
            "C++: space->enforceBounds(state);\n\n"
            "If value < lowerBound → value = lowerBound\n"
            "If value > upperBound → value = upperBound")

        .def("copyState", &discss_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy the integer value from source to destination.\n\n"
            "C++: space->copyState(destination, source);")

        .def("cloneState", &discss_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n\n"
            "C++: space->cloneState(src)->as<DiscreteStateSpace::StateType>()\n\n"
            "Must be freed with freeState().")

        .def("getSerializationLength", &discss_get_serialization_length,
            "Return bytes to serialise one discrete state.\n\n"
            "C++: space->getSerializationLength()\n\n"
            "Returns sizeof(int) = 4 bytes.")

        .def("distance", &discss_distance, py::arg("state1"), py::arg("state2"),
            "Return the distance between two discrete states.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "= |state1.value - state2.value|\n\n"
            "Examples:\n"
            "  distance(state(0), state(3)) = 3.0\n"
            "  distance(state(2), state(2)) = 0.0\n"
            "  distance(state(5), state(1)) = 4.0")

        .def("equalStates", &discss_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if both states have the same integer value.\n\n"
            "C++: space->equalStates(state1, state2)")

        .def("interpolate", &discss_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Interpolate between two discrete states at fraction t.\n\n"
            "C++: space->interpolate(from, to, t, result);\n\n"
            "result.value = round(from.value + t*(to.value - from.value))\n\n"
            "t=0 → from,  t=1 → to,  t=0.5 → rounded midpoint.\n\n"
            "Because states are integers, result is always a valid state.\n"
            "Example: from=0, to=4, t=0.3 → round(0.3*4) = round(1.2) = 1\n"
            "'result' must be pre-allocated.")

        .def("allocState", &discss_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new discrete state. Memory managed by the space.\n\n"
            "C++: space->allocState()->as<DiscreteStateSpace::StateType>()\n\n"
            "State is uninitialised. Set state.value before using.\n"
            "Always call freeState() when done.")

        .def("freeState", &discss_free_state, py::arg("state"),
            "Free a state previously allocated by allocState().\n\n"
            "C++: space->freeState(state);")

        .def("allocDefaultStateSampler", &discss_alloc_default_sampler,
            "Allocate the default discrete sampler.\n\n"
            "C++: space->allocDefaultStateSampler()\n\n"
            "Returns a DiscreteStateSampler.")

        .def("allocStateSampler", &discss_alloc_state_sampler,
            "Allocate a sampler (custom if set, else default).\n\n"
            "C++: space->allocStateSampler()")

        .def("sampleUniform", &discss_sample_uniform, py::arg("state"),
            "Sample a uniformly random integer from [lowerBound, upperBound].\n\n"
            "C++: space->allocDefaultStateSampler()->sampleUniform(state);")

        .def("sampleUniformNear", &discss_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample uniformly from integers within 'distance' of 'near'.\n\n"
            "C++: space->allocDefaultStateSampler()->sampleUniformNear(...);")

        .def("sampleGaussian", &discss_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample from a discrete Gaussian centred at 'mean'.\n\n"
            "C++: space->allocDefaultStateSampler()->sampleGaussian(...);")

        .def("printState", &discss_print_state, py::arg("state"),
            "Return a string showing the integer value.\n\n"
            "C++: space->printState(state, std::cout);")

        .def("printSettings", &discss_print_settings,
            "Return a string describing the space settings.\n\n"
            "C++: space->printSettings(std::cout);")

        .def("registerProjections", &discss_register_projections,
            "Register the default projection. Called automatically by setup().\n\n"
            "C++: space->registerProjections();")

        .def("setup", &discss_setup,
            "Finalise the space.\n\n"
            "C++: space->setup();")

        .def("getName", &DiscreteStateSpace::getName,
            "Return the name of this space.\n"
            "C++: space->getName()")

        .def("setName", &DiscreteStateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n"
            "C++: space->setName(name);")

        .def("__repr__", &discss_repr);
}