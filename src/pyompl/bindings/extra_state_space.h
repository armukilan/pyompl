#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/EmptyStateSpace.h>
#include <ompl/base/spaces/HybridTimeStateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience aliases
using HybTimeST  = HybridTimeStateSpace::StateType;
using WrapperST  = WrapperStateSpace::StateType;

// =============================================================================
// PART 1: EmptyStateSpace
// -----------------------
// EmptyStateSpace is a RealVectorStateSpace with dimension 0.
// It represents a configuration space with NO degrees of freedom.
//
// WHY DOES THIS EXIST?
//   In multi-level planning (e.g. MLRRT, Hierarchical planners),
//   a robot's configuration is decomposed into levels. Some levels
//   may project onto an empty subspace — a placeholder that has
//   no actual degrees of freedom but allows the framework to
//   treat all levels uniformly.
//
// PROPERTIES:
//   getDimension()      → 0  (no degrees of freedom)
//   getMaximumExtent()  → 0  (no distance possible)
//   getMeasure()        → 0  (no volume)
//   setup()             → no-op (nothing to set up)
//
// C++ header: ompl/base/spaces/EmptyStateSpace.h
// C++ class:  ompl::base::EmptyStateSpace
// Inherits:   RealVectorStateSpace(0) → StateSpace
// =============================================================================

static std::string empty_repr(const EmptyStateSpace &s)
{
    return "<EmptyStateSpace name=" + s.getName() + ">";
}

// =============================================================================
// PART 2: HybridTimeStateSpace
// ----------------------------
// HybridTimeStateSpace is like TimeStateSpace but adds a JUMP counter.
// A state has TWO components:
//   position — continuous time value (double)
//   jumps    — discrete jump count (unsigned int)
//
// WHAT ARE JUMPS?
//   In hybrid systems, a robot can have CONTINUOUS dynamics (normal motion)
//   and DISCRETE events (jumps — sudden state changes like a bouncing ball
//   hitting the ground, a gear change, a contact event).
//
//   The jump counter tracks HOW MANY discrete events have occurred.
//   This allows planning in hybrid state spaces where some transitions
//   are continuous and others are instantaneous discrete jumps.
//
// TWO MODES (same as TimeStateSpace):
//   UNBOUNDED (default): no bounds set, all operations are trivial
//   BOUNDED: after setTimeBounds() AND/OR setJumpBounds()
//
// DISTANCE: |position1 - position2| + |jumps1 - jumps2|
// (combines continuous time distance with discrete jump distance)
//
// C++ header: ompl/base/spaces/HybridTimeStateSpace.h
// C++ class:  ompl::base::HybridTimeStateSpace
// Inherits:   StateSpace
// =============================================================================

// HybridTimeStateSpace::StateType helpers
static double hybtime_get_position(const HybTimeST &s)
{
    // C++: state->position
    // Returns the continuous time value.
    return s.position;
}

static void hybtime_set_position(HybTimeST &s, double v)
{
    // C++: state->position = v;
    s.position = v;
}

static unsigned int hybtime_get_jumps(const HybTimeST &s)
{
    // C++: state->jumps
    // Returns the discrete jump counter.
    return s.jumps;
}

static void hybtime_set_jumps(HybTimeST &s, unsigned int v)
{
    // C++: state->jumps = v;
    s.jumps = v;
}

static std::string hybtime_repr(const HybTimeST &s)
{
    return "<HybridTimeState position=" + std::to_string(s.position) +
           " jumps=" + std::to_string(s.jumps) + ">";
}

// HybridTimeStateSampler helpers
static void hybtimesamp_uniform(HybridTimeStateSampler &s, HybTimeST *st)
{
    // C++: sampler->sampleUniform(state);
    // Samples position uniformly from [minTime, maxTime] if bounded (else 0).
    // Samples jumps uniformly from [minJumps, maxJumps] if bounded (else 0).
    s.sampleUniform(static_cast<State *>(st));
}

static void hybtimesamp_uniform_near(HybridTimeStateSampler &s, HybTimeST *st,
                                      const HybTimeST *near, double distance)
{
    // C++: sampler->sampleUniformNear(state, near, distance);
    s.sampleUniformNear(static_cast<State *>(st),
                        static_cast<const State *>(near), distance);
}

static void hybtimesamp_gaussian(HybridTimeStateSampler &s, HybTimeST *st,
                                  const HybTimeST *mean, double stddev)
{
    // C++: sampler->sampleGaussian(state, mean, stdDev);
    s.sampleGaussian(static_cast<State *>(st),
                     static_cast<const State *>(mean), stddev);
}

// HybridTimeStateSpace helpers
static void hybss_set_time_bounds(HybridTimeStateSpace &s, double mn, double mx)
{
    // C++: space->setTimeBounds(minTime, maxTime);
    // Sets the continuous time bounds. Switches to bounded mode for time.
    // After call: isTimeBounded() == True
    s.setTimeBounds(mn, mx);
}

static void hybss_set_jump_bounds(HybridTimeStateSpace &s,
                                   unsigned int mn, unsigned int mx)
{
    // C++: space->setJumpBounds(minJumps, maxJumps);
    // Sets the discrete jump bounds. Switches to bounded mode for jumps.
    // After call: areJumpsBounded() == True
    s.setJumpBounds(mn, mx);
}

static double hybss_get_min_time(const HybridTimeStateSpace &s)
{
    // C++: space->getMinTimeBound()
    return s.getMinTimeBound();
}

static double hybss_get_max_time(const HybridTimeStateSpace &s)
{
    // C++: space->getMaxTimeBound()
    return s.getMaxTimeBound();
}

static double hybss_get_min_jumps(const HybridTimeStateSpace &s)
{
    // C++: space->getMinJumpsBound()
    return s.getMinJumpsBound();
}

static double hybss_get_max_jumps(const HybridTimeStateSpace &s)
{
    // C++: space->getMaxJumpBound()
    return s.getMaxJumpBound();
}

static bool hybss_is_time_bounded(const HybridTimeStateSpace &s)
{
    // C++: space->isTimeBounded()
    return s.isTimeBounded();
}

static bool hybss_are_jumps_bounded(const HybridTimeStateSpace &s)
{
    // C++: space->areJumpsBounded()
    return s.areJumpsBounded();
}

static unsigned int hybss_get_dimension(const HybridTimeStateSpace &s)
{
    // C++: space->getDimension()
    // Returns 1 (time is 1D; jumps are an integer counter, not a dimension).
    return s.getDimension();
}

static double hybss_get_max_extent(const HybridTimeStateSpace &s)
{
    // C++: space->getMaximumExtent()
    // If bounded: maxTime - minTime (+ jump range contribution)
    // If unbounded: 1
    return s.getMaximumExtent();
}

static double hybss_get_measure(const HybridTimeStateSpace &s)
{
    // C++: space->getMeasure()
    return s.getMeasure();
}

static void hybss_enforce_bounds(const HybridTimeStateSpace &s, HybTimeST *st)
{
    // C++: space->enforceBounds(state);
    // Clamps position to [minTime, maxTime] if time bounded.
    // Clamps jumps to [minJumps, maxJumps] if jumps bounded.
    // No-op for unbounded dimensions.
    s.enforceBounds(static_cast<State *>(st));
}

static bool hybss_satisfies_bounds(const HybridTimeStateSpace &s, const HybTimeST *st)
{
    // C++: space->satisfiesBounds(state)
    // Checks position in [minTime, maxTime] AND jumps in [minJumps, maxJumps].
    // Always True for unbounded dimensions.
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void hybss_copy_state(const HybridTimeStateSpace &s,
                              HybTimeST *dst, const HybTimeST *src)
{
    // C++: space->copyState(destination, source);
    // Copies both position AND jumps.
    s.copyState(static_cast<State *>(dst), static_cast<const State *>(src));
}

static HybTimeST *hybss_clone_state(const HybridTimeStateSpace &s,
                                     const HybTimeST *src)
{
    // C++: space->cloneState(src)->as<HybridTimeStateSpace::StateType>()
    return s.cloneState(static_cast<const State *>(src))->as<HybTimeST>();
}

static double hybss_distance(const HybridTimeStateSpace &s,
                               const HybTimeST *s1, const HybTimeST *s2)
{
    // C++: space->distance(state1, state2)
    // = |position1 - position2| + |jumps1 - jumps2|
    // Combines continuous time distance with discrete jump distance.
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool hybss_equal_states(const HybridTimeStateSpace &s,
                                const HybTimeST *s1, const HybTimeST *s2)
{
    // C++: space->equalStates(state1, state2)
    // True only if BOTH position AND jumps are exactly equal.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void hybss_interpolate(const HybridTimeStateSpace &s,
                               const HybTimeST *from, const HybTimeST *to,
                               double t, HybTimeST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // position: linear  result.position = from.position + t*(to.position - from.position)
    // jumps:    rounded result.jumps    = round(from.jumps + t*(to.jumps - from.jumps))
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t, static_cast<State *>(result));
}

static HybTimeST *hybss_alloc_state(HybridTimeStateSpace &s)
{
    // C++: space->allocState()->as<HybridTimeStateSpace::StateType>()
    return s.allocState()->as<HybTimeST>();
}

static void hybss_free_state(HybridTimeStateSpace &s, HybTimeST *st)
{
    // C++: space->freeState(state);
    s.freeState(static_cast<State *>(st));
}

static StateSamplerPtr hybss_alloc_default_sampler(const HybridTimeStateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    return s.allocDefaultStateSampler();
}

static void hybss_sample_uniform(HybridTimeStateSpace &s, HybTimeST *st)
{
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniform(static_cast<State *>(st));
}

static void hybss_sample_uniform_near(HybridTimeStateSpace &s, HybTimeST *st,
                                       const HybTimeST *near, double dist)
{
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniformNear(static_cast<State *>(st),
                               static_cast<const State *>(near), dist);
}

static void hybss_sample_gaussian(HybridTimeStateSpace &s, HybTimeST *st,
                                   const HybTimeST *mean, double stddev)
{
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleGaussian(static_cast<State *>(st),
                            static_cast<const State *>(mean), stddev);
}

static unsigned int hybss_get_serialization_length(const HybridTimeStateSpace &s)
{
    // C++: space->getSerializationLength()
    // = sizeof(double) + sizeof(unsigned int) = 8 + 4 = 12 bytes
    return s.getSerializationLength();
}

static std::string hybss_print_state(const HybridTimeStateSpace &s,
                                      const HybTimeST *st)
{
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string hybss_print_settings(const HybridTimeStateSpace &s)
{
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void hybss_setup(HybridTimeStateSpace &s) { s.setup(); }

static std::string hybss_repr(const HybridTimeStateSpace &s)
{
    return "<HybridTimeStateSpace timeBounded=" +
           std::string(s.isTimeBounded() ? "True" : "False") +
           " jumpsBounded=" +
           std::string(s.areJumpsBounded() ? "True" : "False") + ">";
}

// =============================================================================
// PART 3: WrapperStateSpace
// -------------------------
// WrapperStateSpace is a DECORATOR pattern around any other StateSpace.
// It transparently passes all state space operations to the wrapped space,
// while allowing you to AUGMENT or OVERRIDE specific behaviour.
//
// HOW IT WORKS:
//   WrapperStateSpace wraps a StateSpacePtr (the "inner" space).
//   Every method call (distance, interpolate, etc.) is forwarded to
//   the inner space. The wrapper's StateType holds a pointer to an
//   inner state.
//
// WHY USE THIS?
//   - Add extra information to states without changing the inner space
//   - Override specific methods (e.g. custom distance) while keeping
//     all other behaviour of the wrapped space
//   - Used internally by OMPL for constraint projection spaces
//
// STATE STRUCTURE:
//   WrapperStateSpace::StateType contains:
//     state_ — pointer to the inner space's state
//   Access: wrapperState->getState() → inner State*
//
// NOTE: WrapperStateSpace is rarely used directly by end users.
// It is a base class for specialised wrappers like constraint spaces.
//
// C++ header: ompl/base/spaces/WrapperStateSpace.h
// C++ class:  ompl::base::WrapperStateSpace
// Inherits:   StateSpace
// =============================================================================

// WrapperStateSpace::StateType helpers
static const State *wrapperst_get_state_const(const WrapperST &s)
{
    // C++: state->getState()  (const version)
    // Returns pointer to the inner (wrapped) state.
    return s.getState();
}

static State *wrapperst_get_state(WrapperST &s)
{
    // C++: state->getState()  (non-const version)
    // Returns mutable pointer to the inner (wrapped) state.
    return s.getState();
}

static std::string wrapperst_repr(const WrapperST &s)
{
    return "<WrapperState inner=" +
           std::string(s.getState() ? "set" : "null") + ">";
}

// WrapperStateSpace helpers
static const StateSpacePtr &wrapperss_get_space(const WrapperStateSpace &s)
{
    // C++: space->getSpace()
    // Returns the wrapped (inner) StateSpacePtr.
    // Use this to access the inner space directly.
    return s.getSpace();
}

static State *wrapperss_alloc_state(WrapperStateSpace &s)
{
    // C++: space->allocState()
    // Allocates a WrapperStateSpace::StateType that contains
    // a freshly allocated inner state.
    return s.allocState();
}

static void wrapperss_free_state(WrapperStateSpace &s, State *st)
{
    // C++: space->freeState(state)
    // Frees both the wrapper state AND its inner state.
    s.freeState(st);
}

static void wrapperss_setup(WrapperStateSpace &s)
{
    // C++: space->setup()
    s.setup();
}

static std::string wrapperss_print_settings(const WrapperStateSpace &s)
{
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static std::string wrapperss_repr(const WrapperStateSpace &s)
{
    return "<WrapperStateSpace wrapping=" + s.getName() + ">";
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_extra_state_spaces(py::module_ &m)
{
    // =========================================================================
    // EmptyStateSpace
    // =========================================================================
    py::class_<EmptyStateSpace, RealVectorStateSpace,
               std::shared_ptr<EmptyStateSpace>>(m, "EmptyStateSpace",
        "A state space with zero dimensions — a placeholder.\n\n"
        "C++ header: ompl/base/spaces/EmptyStateSpace.h\n"
        "C++ class:  ompl::base::EmptyStateSpace\n"
        "Inherits:   RealVectorStateSpace(0) → StateSpace\n\n"
        "WHAT IS EmptyStateSpace?\n"
        "A RealVectorStateSpace with dim=0. No degrees of freedom.\n"
        "Used in multi-level planners where some levels project onto\n"
        "an empty subspace (a structural placeholder, not a real space).\n\n"
        "ALL extents are 0:\n"
        "  getDimension()     = 0\n"
        "  getMaximumExtent() = 0\n"
        "  getMeasure()       = 0\n"
        "  setup()            = no-op")

        .def(py::init<>(),
            "Construct an empty state space with zero dimensions.\n\n"
            "C++: EmptyStateSpace space;\n\n"
            "Name is automatically set to 'EmptySpace'.")

        .def("getDimension",
             [](const EmptyStateSpace &s) { return s.getDimension(); },
             "Return 0 — no degrees of freedom.\n"
             "C++: space->getDimension()")

        .def("getMaximumExtent",
             [](const EmptyStateSpace &s) { return s.getMaximumExtent(); },
             "Return 0 — no distance possible.\n"
             "C++: space->getMaximumExtent()")

        .def("getMeasure",
             [](const EmptyStateSpace &s) { return s.getMeasure(); },
             "Return 0 — no volume.\n"
             "C++: space->getMeasure()")

        .def("setup",
             [](EmptyStateSpace &s) { s.setup(); },
             "No-op — nothing to set up for an empty space.\n"
             "C++: space->setup();")

        .def("getName",
             [](const EmptyStateSpace &s) { return s.getName(); },
             "Return 'EmptySpace'.\n"
             "C++: space->getName()")

        .def("__repr__", &empty_repr);

    // =========================================================================
    // HybridTimeStateSpace::StateType
    // =========================================================================
    py::class_<HybTimeST>(m, "HybridTimeState",
        "A state in HybridTimeStateSpace: time + jump count.\n\n"
        "C++ type:   ompl::base::HybridTimeStateSpace::StateType\n"
        "C++ header: ompl/base/spaces/HybridTimeStateSpace.h\n\n"
        "FIELDS:\n"
        "  .position  — continuous time value (double)\n"
        "  .jumps     — discrete jump counter (unsigned int)\n\n"
        "WHAT ARE JUMPS?\n"
        "In hybrid systems, discrete events (jumps) are sudden state\n"
        "changes: a ball bouncing, a gear change, a contact event.\n"
        "The jump counter tracks how many such events have occurred.\n\n"
        "Do NOT construct directly. Use space.allocState().")

        .def_property("position", &hybtime_get_position, &hybtime_set_position,
            "Continuous time value.\n"
            "C++: state->position")

        .def_property("jumps", &hybtime_get_jumps, &hybtime_set_jumps,
            "Discrete jump counter (unsigned int).\n"
            "C++: state->jumps")

        .def("__repr__", &hybtime_repr);

    // =========================================================================
    // HybridTimeStateSampler
    // =========================================================================
    py::class_<HybridTimeStateSampler, StateSampler,
               std::shared_ptr<HybridTimeStateSampler>>(m, "HybridTimeStateSampler",
        "Sampler for HybridTimeStateSpace.\n\n"
        "C++ class:  ompl::base::HybridTimeStateSampler\n\n"
        "Samples both position (time) and jumps independently.\n"
        "Do NOT construct directly. Use space.allocDefaultStateSampler().")

        .def("sampleUniform", &hybtimesamp_uniform, py::arg("state"),
            "Sample position and jumps uniformly from their bounds.\n"
            "Unbounded dimensions always give 0.\n"
            "C++: sampler->sampleUniform(state);")

        .def("sampleUniformNear", &hybtimesamp_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample near 'near' within 'distance'.\n"
            "C++: sampler->sampleUniformNear(state, near, distance);")

        .def("sampleGaussian", &hybtimesamp_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample from Gaussian centred at 'mean'.\n"
            "C++: sampler->sampleGaussian(state, mean, stdDev);");

    // =========================================================================
    // HybridTimeStateSpace
    // =========================================================================
    py::class_<HybridTimeStateSpace, StateSpace,
               std::shared_ptr<HybridTimeStateSpace>>(m, "HybridTimeStateSpace",
        "State space representing hybrid time: continuous time + discrete jumps.\n\n"
        "C++ header: ompl/base/spaces/HybridTimeStateSpace.h\n"
        "C++ class:  ompl::base::HybridTimeStateSpace\n"
        "Inherits:   StateSpace\n\n"
        "WHAT IS HybridTimeStateSpace?\n"
        "Extends TimeStateSpace with a JUMP counter for hybrid systems.\n"
        "A state has TWO components:\n"
        "  position — continuous time (double)\n"
        "  jumps    — discrete jump count (unsigned int)\n\n"
        "DISTANCE:\n"
        "  |position1 - position2| + |jumps1 - jumps2|\n\n"
        "TWO-LEVEL BOUNDING (each independent):\n"
        "  setTimeBounds(min, max)  — bounds on position\n"
        "  setJumpBounds(min, max)  — bounds on jumps\n"
        "  Each can be bounded/unbounded independently.\n\n"
        "USE FOR: hybrid system planning, contact-rich manipulation,\n"
        "         bouncing ball, legged robot contact planning.")

        .def(py::init<>(),
            "Construct an unbounded HybridTimeStateSpace.\n\n"
            "C++: HybridTimeStateSpace space;\n\n"
            "Both time and jumps are unbounded by default.\n"
            "Call setTimeBounds() and/or setJumpBounds() to bound them.")

        .def("setTimeBounds", &hybss_set_time_bounds,
            py::arg("minTime"), py::arg("maxTime"),
            "Set continuous time bounds [minTime, maxTime].\n\n"
            "C++: space->setTimeBounds(minTime, maxTime);\n\n"
            "Switches time to bounded mode: isTimeBounded() → True.\n"
            "Sampler will sample position from [minTime, maxTime].")

        .def("setJumpBounds", &hybss_set_jump_bounds,
            py::arg("minJumps"), py::arg("maxJumps"),
            "Set discrete jump bounds [minJumps, maxJumps].\n\n"
            "C++: space->setJumpBounds(minJumps, maxJumps);\n\n"
            "Switches jumps to bounded mode: areJumpsBounded() → True.\n"
            "Sampler will sample jumps from [minJumps, maxJumps].")

        .def("getMinTimeBound",  &hybss_get_min_time,
            "Return min time bound (0 if unbounded).\n"
            "C++: space->getMinTimeBound()")

        .def("getMaxTimeBound",  &hybss_get_max_time,
            "Return max time bound (0 if unbounded).\n"
            "C++: space->getMaxTimeBound()")

        .def("getMinJumpsBound", &hybss_get_min_jumps,
            "Return min jumps bound (0 if unbounded).\n"
            "C++: space->getMinJumpsBound()")

        .def("getMaxJumpBound",  &hybss_get_max_jumps,
            "Return max jumps bound (0 if unbounded).\n"
            "C++: space->getMaxJumpBound()")

        .def("isTimeBounded",    &hybss_is_time_bounded,
            "Return True if setTimeBounds() has been called.\n"
            "C++: space->isTimeBounded()")

        .def("areJumpsBounded",  &hybss_are_jumps_bounded,
            "Return True if setJumpBounds() has been called.\n"
            "C++: space->areJumpsBounded()")

        .def("getDimension",     &hybss_get_dimension,
            "Return 1 (time is 1D; jumps are a counter, not a dimension).\n"
            "C++: space->getDimension()")

        .def("getMaximumExtent", &hybss_get_max_extent,
            "Return max extent (maxTime-minTime if bounded, else 1).\n"
            "C++: space->getMaximumExtent()")

        .def("getMeasure",       &hybss_get_measure,
            "Return measure of the space.\n"
            "C++: space->getMeasure()")

        .def("satisfiesBounds",  &hybss_satisfies_bounds, py::arg("state"),
            "Return True if position and jumps are within their bounds.\n"
            "Always True for unbounded dimensions.\n"
            "C++: space->satisfiesBounds(state)")

        .def("enforceBounds",    &hybss_enforce_bounds, py::arg("state"),
            "Clamp position and jumps to their bounds. No-op if unbounded.\n"
            "C++: space->enforceBounds(state);")

        .def("copyState",        &hybss_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy both position AND jumps from source to destination.\n"
            "C++: space->copyState(destination, source);")

        .def("cloneState",       &hybss_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n"
            "C++: space->cloneState(src)->as<HybridTimeStateSpace::StateType>()")

        .def("getSerializationLength", &hybss_get_serialization_length,
            "Return bytes to serialise: sizeof(double)+sizeof(uint) = 12.\n"
            "C++: space->getSerializationLength()")

        .def("distance",         &hybss_distance,
            py::arg("state1"), py::arg("state2"),
            "Return |pos1-pos2| + |jumps1-jumps2|.\n"
            "C++: space->distance(state1, state2)")

        .def("equalStates",      &hybss_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if BOTH position AND jumps are exactly equal.\n"
            "C++: space->equalStates(state1, state2)")

        .def("interpolate",      &hybss_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Linear interpolation for position, rounded for jumps.\n"
            "C++: space->interpolate(from, to, t, result);")

        .def("allocState",       &hybss_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new HybridTimeState.\n"
            "C++: space->allocState()->as<HybridTimeStateSpace::StateType>()")

        .def("freeState",        &hybss_free_state, py::arg("state"),
            "Free a previously allocated state.\n"
            "C++: space->freeState(state);")

        .def("allocDefaultStateSampler", &hybss_alloc_default_sampler,
            "Allocate the default sampler.\n"
            "C++: space->allocDefaultStateSampler()")

        .def("sampleUniform",     &hybss_sample_uniform,     py::arg("state"),
            "Sample uniformly within bounds (or 0 if unbounded).\n"
            "C++: space->allocDefaultStateSampler()->sampleUniform(state);")

        .def("sampleUniformNear", &hybss_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample near 'near' within 'distance'.\n"
            "C++: space->allocDefaultStateSampler()->sampleUniformNear(...);")

        .def("sampleGaussian",    &hybss_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample from Gaussian centred at 'mean'.\n"
            "C++: space->allocDefaultStateSampler()->sampleGaussian(...);")

        .def("printState",       &hybss_print_state, py::arg("state"),
            "Return a string showing position and jumps.\n"
            "C++: space->printState(state, std::cout);")

        .def("printSettings",    &hybss_print_settings,
            "Return a string describing space settings.\n"
            "C++: space->printSettings(std::cout);")

        .def("setup",            &hybss_setup,
            "Finalise the space.\n"
            "C++: space->setup();")

        .def("getName",          &HybridTimeStateSpace::getName,
            "Return the name.\nC++: space->getName()")

        .def("setName",          &HybridTimeStateSpace::setName, py::arg("name"),
            "Set the name.\nC++: space->setName(name);")

        .def("__repr__",         &hybss_repr);

    // =========================================================================
    // WrapperStateSpace::StateType
    // =========================================================================
    py::class_<WrapperST, State, std::unique_ptr<WrapperST, py::nodelete>>(
        m, "WrapperState",
        "A state in WrapperStateSpace — wraps an inner state.\n\n"
        "C++ type:   ompl::base::WrapperStateSpace::StateType\n"
        "C++ header: ompl/base/spaces/WrapperStateSpace.h\n\n"
        "Contains a pointer to an inner state from the wrapped space.\n"
        "Do NOT construct directly. Use space.allocState().")

        .def("getState",
             py::overload_cast<>(&WrapperST::getState),
             py::return_value_policy::reference_internal,
             "Return the inner (wrapped) state.\n"
             "C++: state->getState()\n\n"
             "This is the actual state of the wrapped space.\n"
             "Cast it to the appropriate type to read/write values.")

        .def("__repr__", &wrapperst_repr);

    // =========================================================================
    // WrapperStateSpace
    // =========================================================================
    py::class_<WrapperStateSpace, StateSpace,
               std::shared_ptr<WrapperStateSpace>>(m, "WrapperStateSpace",
        "A state space that transparently wraps another state space.\n\n"
        "C++ header: ompl/base/spaces/WrapperStateSpace.h\n"
        "C++ class:  ompl::base::WrapperStateSpace\n"
        "Inherits:   StateSpace\n\n"
        "WHAT IS WrapperStateSpace?\n"
        "A DECORATOR pattern — wraps any StateSpacePtr and forwards\n"
        "ALL operations to the inner space transparently.\n\n"
        "WHY USE IT?\n"
        "  - Override specific methods while keeping all others\n"
        "  - Add extra information to states\n"
        "  - Base class for constraint spaces in OMPL\n\n"
        "STATE STRUCTURE:\n"
        "  WrapperState.getState() → pointer to inner state\n"
        "  The inner state belongs to the wrapped space.\n\n"
        "ALL methods delegate to the wrapped space:\n"
        "  distance(s1,s2) → wrappedSpace->distance(s1.inner, s2.inner)\n"
        "  interpolate(...)→ wrappedSpace->interpolate(...)\n"
        "  etc.")

        .def(py::init<const StateSpacePtr &>(), py::arg("space"),
            "Wrap an existing state space.\n\n"
            "C++: WrapperStateSpace wrapper(space);\n\n"
            "All operations are forwarded to 'space'.")

        .def("getSpace",     &wrapperss_get_space,
            py::return_value_policy::reference_internal,
            "Return the wrapped (inner) StateSpacePtr.\n\n"
            "C++: space->getSpace()\n\n"
            "Use to access the inner space directly for configuration.")

        .def("allocState",   &wrapperss_alloc_state,
            py::return_value_policy::reference,
            "Allocate a WrapperState containing a new inner state.\n\n"
            "C++: space->allocState()\n\n"
            "The returned state wraps a freshly allocated inner state.")

        .def("freeState",    &wrapperss_free_state, py::arg("state"),
            "Free wrapper state AND its inner state.\n\n"
            "C++: space->freeState(state);")

        .def("getDimension",
             [](const WrapperStateSpace &s) { return s.getDimension(); },
             "Return dimension of the wrapped space.\n"
             "C++: space->getDimension()")

        .def("getMaximumExtent",
             [](const WrapperStateSpace &s) { return s.getMaximumExtent(); },
             "Return max extent of the wrapped space.\n"
             "C++: space->getMaximumExtent()")

        .def("isCompound",
             [](const WrapperStateSpace &s) { return s.isCompound(); },
             "Delegate to wrapped space.\nC++: space->isCompound()")

        .def("isDiscrete",
             [](const WrapperStateSpace &s) { return s.isDiscrete(); },
             "Delegate to wrapped space.\nC++: space->isDiscrete()")

        .def("isMetricSpace",
             [](const WrapperStateSpace &s) { return s.isMetricSpace(); },
             "Delegate to wrapped space.\nC++: space->isMetricSpace()")

        .def("printSettings", &wrapperss_print_settings,
             "Return settings string of wrapped space.\n"
             "C++: space->printSettings(std::cout);")

        .def("setup",        &wrapperss_setup,
             "Call setup on wrapped space.\nC++: space->setup();")

        .def("getName",
             [](const WrapperStateSpace &s) { return s.getName(); },
             "Return name of wrapped space.\nC++: space->getName()")

        .def("setName",
             [](WrapperStateSpace &s, const std::string &n) { s.setName(n); },
             py::arg("name"),
             "Set name on wrapped space.\nC++: space->setName(name);")

        .def("__repr__", &wrapperss_repr);
}