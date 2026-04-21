#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience alias for the state type
using SO2ST = SO2StateSpace::StateType;

// =============================================================================
// WHY FREE FUNCTIONS?
// MSVC (Windows compiler) does not reliably handle C++ lambdas inside heavily
// templated pybind11 .def() chains. We use plain static free functions instead.
// Each function below wraps one OMPL C++ method. The naming convention is:
//   so2st_*   → SO2StateSpace::StateType  (the state object)
//   so2ss_*   → SO2StateSpace             (the space object)
//   so2samp_* → SO2StateSampler           (the sampler object)
// =============================================================================

// -----------------------------------------------------------------------------
// SO2StateSpace::StateType helpers
//
// WHAT IS SO2StateSpace::StateType?
// A state in SO(2) is just ONE double: an angle in (-π, π].
// The C++ struct has a single public field:  double value;
// In Python we expose this as .value property + setIdentity().
// -----------------------------------------------------------------------------

static double so2st_get_value(const SO2ST &s)
{
    // C++: state->value
    // Returns the angle stored in this state (in radians, range (-π, π]).
    return s.value;
}

static void so2st_set_value(SO2ST &s, double v)
{
    // C++: state->value = v;
    // Directly sets the angle. Does NOT enforce bounds — call
    // space.enforceBounds(state) afterwards if v may be outside (-π, π].
    s.value = v;
}

static void so2st_set_identity(SO2ST &s)
{
    // C++: state->setIdentity();
    // Sets the state to the identity rotation: value = 0.0.
    // Corresponds to "no rotation" or "zero angle".
    s.setIdentity();
}

static std::string so2st_repr(const SO2ST &s)
{
    return "<SO2State value=" + std::to_string(s.value) + ">";
}

// -----------------------------------------------------------------------------
// SO2StateSampler helpers
// -----------------------------------------------------------------------------

static void so2samp_sample_uniform(SO2StateSampler &s, SO2ST *st)
{
    // C++: sampler->sampleUniform(state);
    // Samples an angle uniformly at random from (-π, π].
    // The sampled value is always a valid SO(2) state.
    s.sampleUniform(static_cast<State *>(st));
}

static void so2samp_sample_uniform_near(SO2StateSampler &s, SO2ST *st,
                                         const SO2ST *near, double distance)
{
    // C++: sampler->sampleUniformNear(state, near, distance);
    // Samples an angle uniformly from the arc
    //   [near->value - distance, near->value + distance]
    // on the circle. The result is wrapped to (-π, π].
    // 'distance' is in radians.
    s.sampleUniformNear(static_cast<State *>(st),
                        static_cast<const State *>(near),
                        distance);
}

static void so2samp_sample_gaussian(SO2StateSampler &s, SO2ST *st,
                                     const SO2ST *mean, double stddev)
{
    // C++: sampler->sampleGaussian(state, mean, stdDev);
    // Samples an angle from a Gaussian (wrapped normal distribution)
    // centred at mean->value with the given standard deviation (radians).
    // The result is wrapped to (-π, π].
    s.sampleGaussian(static_cast<State *>(st),
                     static_cast<const State *>(mean),
                     stddev);
}

// -----------------------------------------------------------------------------
// SO2StateSpace helpers
// -----------------------------------------------------------------------------

static unsigned int so2ss_get_dimension(const SO2StateSpace &s)
{
    // C++: space->getDimension()
    // Always returns 1 for SO(2). SO(2) is a 1-dimensional manifold
    // (a circle), even though it is embedded in 2D Euclidean space.
    return s.getDimension();
}

static double so2ss_get_maximum_extent(const SO2StateSpace &s)
{
    // C++: space->getMaximumExtent()
    // Returns the maximum distance between any two SO(2) states.
    // Because distance is the shortest arc on the circle, the maximum
    // is π (half the circumference of a unit circle), achieved when
    // the two states are directly opposite each other.
    return s.getMaximumExtent();
}

static double so2ss_get_measure(const SO2StateSpace &s)
{
    // C++: space->getMeasure()
    // Returns the measure (length) of the state space.
    // For SO(2) this is 2π — the full circumference of the unit circle.
    return s.getMeasure();
}

static void so2ss_enforce_bounds(const SO2StateSpace &s, SO2ST *st)
{
    // C++: space->enforceBounds(state);
    // Wraps the angle to lie in (-π, π].
    // Example: if state->value = 4.0 (> π), it gets wrapped to 4.0 - 2π ≈ -2.28.
    // This is called "angle normalisation" and is needed because SO(2) is
    // periodic — angles 0 and 2π represent the same rotation.
    s.enforceBounds(static_cast<State *>(st));
}

static bool so2ss_satisfies_bounds(const SO2StateSpace &s, const SO2ST *st)
{
    // C++: space->satisfiesBounds(state)
    // Returns True if state->value is in (-π, π].
    // Does NOT modify the state — use enforceBounds to fix it.
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void so2ss_copy_state(const SO2StateSpace &s, SO2ST *dst, const SO2ST *src)
{
    // C++: space->copyState(destination, source);
    // Copies the angle value from src to dst.
    // Memory of src and dst must not overlap.
    s.copyState(static_cast<State *>(dst),
                static_cast<const State *>(src));
}

static unsigned int so2ss_get_serialization_length(const SO2StateSpace &s)
{
    // C++: space->getSerializationLength()
    // Returns the number of bytes needed to serialise one SO(2) state.
    // = sizeof(double) = 8 bytes (just the one angle value).
    return s.getSerializationLength();
}

static double so2ss_distance(const SO2StateSpace &s,
                              const SO2ST *s1, const SO2ST *s2)
{
    // C++: space->distance(state1, state2)
    // Returns the SHORTEST arc distance between two angles on the circle.
    // This is NOT |s1 - s2|. It accounts for angle wrapping.
    // Example: distance(angle=3.0, angle=-3.0) ≈ 0.28, not 6.0,
    //          because going the short way round the circle is shorter.
    // Formula: min(|a - b|, 2π - |a - b|)
    // Always in [0, π].
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool so2ss_equal_states(const SO2StateSpace &s,
                                const SO2ST *s1, const SO2ST *s2)
{
    // C++: space->equalStates(state1, state2)
    // Returns True if both states have exactly the same angle value.
    // Exact floating-point comparison — not approximate.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void so2ss_interpolate(const SO2StateSpace &s,
                               const SO2ST *from, const SO2ST *to,
                               double t, SO2ST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // Interpolates between two angles taking the SHORTEST path around
    // the circle. t=0 → from, t=1 → to, t=0.5 → halfway along shortest arc.
    // Result is always in (-π, π].
    // Example: interpolate(angle=-3.0, angle=3.0, t=0.5) gives ≈ π or -π
    //          (the point directly between them going the short way).
    // This is NOT simple linear interpolation of the raw values.
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}

static SO2ST *so2ss_alloc_state(SO2StateSpace &s)
{
    // C++: space->allocState()->as<SO2StateSpace::StateType>()
    // Allocates a new SO(2) state on the heap.
    // Memory is managed by the space — always call freeState() when done.
    // The allocated state is uninitialised; set state.value before using it.
    return s.allocState()->as<SO2ST>();
}

static void so2ss_free_state(SO2StateSpace &s, SO2ST *st)
{
    // C++: space->freeState(state);
    // Frees memory allocated by allocState().
    // Always call this when done with a state to avoid memory leaks.
    s.freeState(static_cast<State *>(st));
}

static SO2ST *so2ss_clone_state(const SO2StateSpace &s, const SO2ST *src)
{
    // C++: space->cloneState(src)->as<SO2StateSpace::StateType>()
    // Allocates a new state and copies src into it in one call.
    // Equivalent to: dst = allocState(); copyState(dst, src);
    // Must be freed with freeState().
    return s.cloneState(static_cast<const State *>(src))->as<SO2ST>();
}

static StateSamplerPtr so2ss_alloc_default_sampler(const SO2StateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    // Returns a SO2StateSampler for this space.
    // Use this to randomly sample SO(2) states for collision checking,
    // exploration, or initialisation.
    return s.allocDefaultStateSampler();
}

static StateSamplerPtr so2ss_alloc_state_sampler(const SO2StateSpace &s)
{
    // C++: space->allocStateSampler()
    // Returns the sampler set by setStateSamplerAllocator(), or the
    // default sampler if no custom allocator has been set.
    return s.allocStateSampler();
}

static std::string so2ss_print_state(const SO2StateSpace &s, const SO2ST *st)
{
    // C++: space->printState(state, std::cout);
    // Returns a human-readable string representation of the state.
    // Typically prints the angle value in radians.
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string so2ss_print_settings(const SO2StateSpace &s)
{
    // C++: space->printSettings(std::cout);
    // Returns a string describing the space configuration:
    // name, type, dimension, maximum extent, etc.
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void so2ss_register_projections(SO2StateSpace &s)
{
    // C++: space->registerProjections();
    // Registers the default projection for SO(2).
    // A projection maps states to R^k for use by grid-based planners
    // like KPIECE. For SO(2), the default projects to R^1 (just the angle).
    // Called automatically by setup().
    s.registerProjections();
}

static void so2ss_setup(SO2StateSpace &s)
{
    // C++: space->setup();
    // Finalises the space: registers projections, computes valid segment
    // lengths, and builds internal lookup tables.
    // SpaceInformation calls this automatically — but call it manually
    // if you use the space standalone.
    s.setup();
}

// static std::vector<double> so2ss_copy_to_reals(const SO2StateSpace &s, const SO2ST *st)
// {
//     // C++: space->copyToReals(reals, source);
//     // Extracts all double values from the state into a std::vector.
//     // For SO(2) this returns a 1-element list: [angle].
    
//     // Changed this line
//     std::vector<double> reals;
//     // std::vector<double> reals(s.getDimension());
//     s.copyToReals(reals, static_cast<const State *>(st));
//     return reals;
// }
static std::vector<double> so2ss_copy_to_reals(const SO2StateSpace &s, const SO2ST *st)
{
    // computeLocations() must have been called (via setup()) before
    // copyToReals works. Cast away const to call it if needed.
    const_cast<SO2StateSpace &>(s).computeLocations();
    std::vector<double> reals;
    s.copyToReals(reals, static_cast<const State *>(st));
    return reals;
}

static void so2ss_copy_from_reals(const SO2StateSpace &s,
                                   SO2ST *dst,
                                   const std::vector<double> &reals)
{
    // C++: space->copyFromReals(destination, reals);
    // Sets the state values from a std::vector of doubles.
    // For SO(2), reals must have exactly 1 element: the angle.
    s.copyFromReals(static_cast<State *>(dst), reals);
}

static std::string so2ss_repr(const SO2StateSpace &s)
{
    return "<SO2StateSpace name=" + s.getName() + ">";
}

// =============================================================================
// Binding function — called from _core.cpp
// =============================================================================

inline void bind_so2_state_space(py::module_ &m)
{
    // -------------------------------------------------------------------------
    // SO2StateSpace::StateType
    //
    // WHAT IS SO2StateSpace::StateType?
    // ----------------------------------
    // The state for SO(2) is the simplest possible state: a single angle.
    // It inherits from ompl::base::State and adds one public field:
    //
    //   double value;   ← the angle in radians, range (-π, π]
    //
    // In C++ you access it like:
    //   auto *s = space->allocState()->as<SO2StateSpace::StateType>();
    //   s->value = 1.57;   // 90 degrees
    //
    // In Python (pyompl):
    //   s = space.allocState()
    //   s.value = 1.57
    // -------------------------------------------------------------------------
    py::class_<SO2ST>(m, "SO2State",
        "A single state in SO(2) — one angle in radians.\n\n"
        "C++ type: ompl::base::SO2StateSpace::StateType\n"
        "C++ header: ompl/base/spaces/SO2StateSpace.h\n\n"
        "SO(2) is the group of 2D rotations. A state here is just ONE angle\n"
        "in the range (-π, π]. It represents a heading, orientation, or yaw.\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocState() to get one, space.freeState() when done.\n\n"
        "Fields:\n"
        "  .value  — the angle in radians, range (-π, π]")

        .def_property("value",
            &so2st_get_value,
            &so2st_set_value,
            "The angle stored in this state (radians, range (-π, π]).\n"
            "C++: state->value\n\n"
            "Reading:  angle = state.value\n"
            "Writing:  state.value = 1.57\n\n"
            "Note: Setting a value outside (-π, π] does NOT automatically\n"
            "normalise it. Call space.enforceBounds(state) to normalise.")

        .def("setIdentity", &so2st_set_identity,
            "Set this state to the identity rotation (zero angle).\n\n"
            "C++: state->setIdentity();\n\n"
            "Sets state.value = 0.0. This represents 'no rotation' —\n"
            "the robot is facing the reference direction.")

        .def("__repr__", &so2st_repr);

    // -------------------------------------------------------------------------
    // SO2StateSampler
    //
    // WHAT IS SO2StateSampler?
    // -------------------------
    // A sampler that generates random SO(2) states (random angles).
    // It knows about the circular topology of SO(2) — samples wrap
    // correctly around the circle.
    //
    // You do NOT construct this directly. Get one from the space:
    //   C++:    auto sampler = space->allocDefaultStateSampler();
    //   Python: sampler = space.allocDefaultStateSampler()
    // -------------------------------------------------------------------------
    py::class_<SO2StateSampler, StateSampler,
               std::shared_ptr<SO2StateSampler>>(m, "SO2StateSampler",
        "Sampler for SO(2) — generates random angles.\n\n"
        "C++ class:  ompl::base::SO2StateSampler\n"
        "C++ header: ompl/base/spaces/SO2StateSpace.h\n\n"
        "Knows about the circular topology of SO(2):\n"
        "samples wrap correctly around the circle.\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocDefaultStateSampler() to get one.")

        .def("sampleUniform", &so2samp_sample_uniform,
            py::arg("state"),
            "Sample a uniformly random angle from (-π, π].\n\n"
            "C++: sampler->sampleUniform(state);\n\n"
            "Every angle on the circle is equally likely.\n"
            "The result is always a valid SO(2) state.\n"
            "Used by planners to explore the rotation space.")

        .def("sampleUniformNear", &so2samp_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample a random angle near 'near' within 'distance' radians.\n\n"
            "C++: sampler->sampleUniformNear(state, near, distance);\n\n"
            "Samples uniformly from the arc:\n"
            "  [near.value - distance, near.value + distance]\n"
            "The result is wrapped to (-π, π].\n"
            "'distance' is in radians. It is the half-width of the arc.\n"
            "Used by planners that expand locally (RRT, etc.).")

        .def("sampleGaussian", &so2samp_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample an angle from a wrapped Gaussian centred at 'mean'.\n\n"
            "C++: sampler->sampleGaussian(state, mean, stdDev);\n\n"
            "Generates angles more likely to be near mean.value,\n"
            "falling off with standard deviation stdDev (radians).\n"
            "The result is wrapped to (-π, π].\n"
            "Used by planners that bias sampling toward certain regions.");

    // -------------------------------------------------------------------------
    // SO2StateSpace
    //
    // WHAT IS SO2StateSpace?
    // -----------------------
    // SO(2) = Special Orthogonal group in 2D = the space of all 2D rotations.
    // Geometrically it is a CIRCLE, not a line segment.
    //
    // Key difference from RealVectorStateSpace(1):
    //   - In R^1, angle 3.14 and angle -3.14 are far apart (distance 6.28)
    //   - In SO(2), they are the SAME point (distance 0) — they both mean π
    //
    // All operations (distance, interpolate, enforceBounds) respect this
    // circular topology via angle wrapping.
    //
    // Use SO2StateSpace for: robot heading, joint that rotates freely,
    //                        any angle without limits.
    // Use RealVectorStateSpace(1) for: a joint with hard stops (e.g. [-π, π]).
    //
    // C++ header: ompl/base/spaces/SO2StateSpace.h
    // C++ class:  ompl::base::SO2StateSpace
    // -------------------------------------------------------------------------
    py::class_<SO2StateSpace, StateSpace,
               std::shared_ptr<SO2StateSpace>>(m, "SO2StateSpace",
        "State space representing SO(2) — the group of 2D rotations.\n\n"
        "C++ header: ompl/base/spaces/SO2StateSpace.h\n"
        "C++ class:  ompl::base::SO2StateSpace\n"
        "Inherits from: StateSpace (ompl::base::StateSpace)\n\n"
        "SO(2) is a CIRCLE, not a line. A state is a single angle in (-π, π].\n"
        "All operations respect circular topology (angle wrapping):\n"
        "  - distance(3.0, -3.0) ≈ 0.28  (short arc), not 6.0\n"
        "  - enforceBounds wraps 7.0 → 0.72 (7.0 - 2π)\n"
        "  - interpolate goes the short way around the circle\n\n"
        "Use for: robot heading, freely-rotating joints, yaw angle.\n"
        "Do NOT use for joints with hard mechanical stops — use\n"
        "RealVectorStateSpace(1) with explicit bounds for those.")

        .def(py::init<>(),
            "Construct an SO(2) state space.\n\n"
            "C++: auto space = std::make_shared<SO2StateSpace>();\n\n"
            "No bounds needed — SO(2) is inherently bounded by its\n"
            "circular topology. The valid range (-π, π] is fixed.")

        // --- dimension ---
        .def("getDimension", &so2ss_get_dimension,
            "Return the dimension of SO(2): always 1.\n\n"
            "C++: space->getDimension()\n\n"
            "SO(2) is a 1-dimensional manifold (a circle),\n"
            "even though it lives in 2D Euclidean space.\n"
            "Compare: SE(2) = R^2 × SO(2) has dimension 3.")

        // --- extent and measure ---
        .def("getMaximumExtent", &so2ss_get_maximum_extent,
            "Return the maximum distance between any two SO(2) states.\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "Always returns π (≈ 3.14159...).\n"
            "This is the distance between two angles that are exactly\n"
            "opposite on the circle — the longest shortest arc.\n"
            "Used by planners to normalise distances.")

        .def("getMeasure", &so2ss_get_measure,
            "Return the measure (total length) of SO(2).\n\n"
            "C++: space->getMeasure()\n\n"
            "Always returns 2π (≈ 6.28318...) — the full circumference\n"
            "of the unit circle. This is the 'volume' of the space.")

        // # --- bounds ---
        .def("enforceBounds", &so2ss_enforce_bounds, py::arg("state"),
            "Wrap the angle to lie in (-π, π]. Modifies state in place.\n\n"
            "C++: space->enforceBounds(state);\n\n"
            "Because SO(2) is periodic, any angle is equivalent to\n"
            "some angle in (-π, π] by adding/subtracting 2π.\n"
            "Examples:\n"
            "  4.0   → 4.0 - 2π ≈ -2.28\n"
            "  -4.0  → -4.0 + 2π ≈  2.28\n"
            "  7.0   → 7.0 - 2π  ≈  0.72\n"
            "  -7.0  → -7.0 + 2π ≈ -0.72\n"
            "  3.14  → 3.14 (already in range)\n"
            "Called automatically when planners sample or propagate states.")

        .def("satisfiesBounds", &so2ss_satisfies_bounds, py::arg("state"),
            "Return True if state.value is in (-π, π].\n\n"
            "C++: space->satisfiesBounds(state)\n\n"
            "Does NOT modify the state. Use enforceBounds() to fix it.\n"
            "Note: the interval is OPEN on the left: -π is NOT valid,\n"
            "but π IS valid (the range is (-π, π], not [-π, π]).")

        // # --- state operations ---
        .def("copyState", &so2ss_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy the angle value from source into destination.\n\n"
            "C++: space->copyState(destination, source);\n\n"
            "Memory of source and destination must not overlap.\n"
            "After this call: destination.value == source.value")

        .def("cloneState", &so2ss_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n\n"
            "C++: space->cloneState(src)->as<SO2StateSpace::StateType>()\n\n"
            "Equivalent to: dst = allocState(); copyState(dst, src).\n"
            "Must be freed with freeState().")

        .def("copyToReals", &so2ss_copy_to_reals, py::arg("source"),
            "Return the state's angle as a 1-element Python list.\n\n"
            "C++: space->copyToReals(reals, source);\n\n"
            "Returns [state.value]. Useful for generic code that\n"
            "works with any state space via lists of doubles.")

        .def("copyFromReals", &so2ss_copy_from_reals,
            py::arg("destination"), py::arg("reals"),
            "Set the state's angle from a 1-element Python list.\n\n"
            "C++: space->copyFromReals(destination, reals);\n\n"
            "'reals' must have exactly 1 element: the angle in radians.\n"
            "Does NOT automatically enforce bounds.")

        // # --- serialization ---
        .def("getSerializationLength", &so2ss_get_serialization_length,
            "Return the number of bytes to serialise one SO(2) state.\n\n"
            "C++: space->getSerializationLength()\n\n"
            "Always returns 8 (= sizeof(double)).\n"
            "The state is just one double, so serialisation is trivial.")

        // # --- distance and equality ---
        .def("distance", &so2ss_distance, py::arg("state1"), py::arg("state2"),
            "Return the shortest arc distance between two angles.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "This is NOT |a - b|. It accounts for the circular topology:\n"
            "  distance = min(|a - b|, 2π - |a - b|)\n\n"
            "Always in [0, π].\n\n"
            "Examples:\n"
            "  distance(0.0, 1.0)   = 1.0\n"
            "  distance(3.0, -3.0)  ≈ 0.28   (short way round)\n"
            "  distance(0.0, π)     = π       (opposite sides)\n"
            "  distance(π, -π)      = 0.0     (same point!)\n\n"
            "This metric is what makes SO(2) different from R^1.")

        .def("equalStates", &so2ss_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if both states have exactly the same angle.\n\n"
            "C++: space->equalStates(state1, state2)\n\n"
            "Exact floating-point comparison.\n"
            "Note: π and -π are NOT equal here even though they represent\n"
            "the same point geometrically. Normalise both states first\n"
            "with enforceBounds() to avoid this edge case.")

        // # --- interpolation ---
        .def("interpolate", &so2ss_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Interpolate between two angles along the SHORTEST arc.\n\n"
            "C++: space->interpolate(from, to, t, result);\n\n"
            "t=0.0 → result == from\n"
            "t=1.0 → result == to\n"
            "t=0.5 → result is halfway along the shortest arc\n\n"
            "This is NOT linear interpolation of raw values.\n"
            "It always takes the shorter path around the circle.\n\n"
            "Example:\n"
            "  from=3.0, to=-3.0: shortest arc goes through ±π.\n"
            "  from=0.0, to=2.0:  goes directly, result at t=0.5 is 1.0.\n\n"
            "'result' must be a pre-allocated state.")

        // # --- allocation ---
        .def("allocState", &so2ss_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new SO(2) state. Memory managed by the space.\n\n"
            "C++: space->allocState()->as<SO2StateSpace::StateType>()\n\n"
            "The state is uninitialised — set state.value before using it.\n"
            "Always call freeState() when done to avoid memory leaks.")

        .def("freeState", &so2ss_free_state, py::arg("state"),
            "Free a state previously allocated by allocState().\n\n"
            "C++: space->freeState(state);\n\n"
            "Always call this when done with a state.")

        // # --- sampler ---
        .def("allocDefaultStateSampler", &so2ss_alloc_default_sampler,
            "Allocate the default SO(2) sampler for this space.\n\n"
            "C++: space->allocDefaultStateSampler()\n\n"
            "Returns a SO2StateSampler. Use its methods to generate\n"
            "random SO(2) states for planning or testing.")

        .def("allocStateSampler", &so2ss_alloc_state_sampler,
            "Allocate a sampler (custom if set, else default).\n\n"
            "C++: space->allocStateSampler()\n\n"
            "Returns the custom sampler set via setStateSamplerAllocator(),\n"
            "or the default SO2StateSampler if none is set.")

        // # --- debug / print ---
        .def("printState", &so2ss_print_state, py::arg("state"),
            "Return a string representation of the given state.\n\n"
            "C++: space->printState(state, std::cout);\n\n"
            "Prints the angle value in radians. Useful for debugging.")

        .def("printSettings", &so2ss_print_settings,
            "Return a string describing this space's settings.\n\n"
            "C++: space->printSettings(std::cout);\n\n"
            "Prints: name, type, dimension, maximum extent, etc.")

        // # --- projections and setup ---
        .def("registerProjections", &so2ss_register_projections,
            "Register the default projection for SO(2).\n\n"
            "C++: space->registerProjections();\n\n"
            "A projection maps SO(2) states to R^k for use by\n"
            "grid-based planners like KPIECE1.\n"
            "For SO(2), the default projection maps to R^1 (just the angle).\n"
            "Called automatically by setup().")

        .def("setup", &so2ss_setup,
            "Finalise the space. Call after construction.\n\n"
            "C++: space->setup();\n\n"
            "Registers default projections, computes valid segment lengths,\n"
            "and builds internal lookup tables.\n"
            "SpaceInformation calls this automatically — but call it manually\n"
            "if using the space standalone (e.g., for testing).")

        // # --- name (inherited) ---
        .def("getName", &SO2StateSpace::getName,
            "Return the name of this space.\n\n"
            "C++: space->getName()\n\n"
            "Default name is 'SO2' + auto-generated suffix.")

        .def("setName", &SO2StateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n\n"
            "C++: space->setName(name);\n\n"
            "Useful for debugging and identifying spaces in compound setups.")

        .def("__repr__", &so2ss_repr);
}