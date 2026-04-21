#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience alias
using SO3ST = SO3StateSpace::StateType;

// =============================================================================
// WHAT IS SO(3)?
// --------------
// SO(3) = Special Orthogonal Group in 3 dimensions.
// It is the space of ALL possible 3D rotations.
//
// OMPL represents SO(3) states as UNIT QUATERNIONS: (x, y, z, w)
// where x^2 + y^2 + z^2 + w^2 = 1.
//
// WHY QUATERNIONS instead of Euler angles?
//   - No gimbal lock (singularities that break interpolation)
//   - Smooth, consistent distance metric (angle between quaternions)
//   - Efficient SLERP interpolation
//
// A unit quaternion q = (x, y, z, w) represents:
//   - axis of rotation: (x, y, z) / sin(angle/2)
//   - amount of rotation: angle = 2 * acos(w)
//
// KEY PROPERTIES:
//   - q and -q represent the SAME rotation (double cover of SO(3))
//   - Identity rotation: (0, 0, 0, 1)
//   - Distance is the angle between orientations, in [0, π/2]
//   - Interpolation uses SLERP (Spherical Linear Interpolation)
//
// USE FOR: 3D robot orientation, joint that rotates freely in 3D,
//          camera orientation, drone attitude (roll/pitch/yaw combined)
//
// C++ header: ompl/base/spaces/SO3StateSpace.h
// C++ class:  ompl::base::SO3StateSpace
// =============================================================================

// -----------------------------------------------------------------------------
// SO3StateSpace::StateType helpers
// -----------------------------------------------------------------------------

static double so3st_get_x(const SO3ST &s) { return s.x; }
static double so3st_get_y(const SO3ST &s) { return s.y; }
static double so3st_get_z(const SO3ST &s) { return s.z; }
static double so3st_get_w(const SO3ST &s) { return s.w; }

static void so3st_set_x(SO3ST &s, double v) { s.x = v; }
static void so3st_set_y(SO3ST &s, double v) { s.y = v; }
static void so3st_set_z(SO3ST &s, double v) { s.z = v; }
static void so3st_set_w(SO3ST &s, double v) { s.w = v; }

static void so3st_set_axis_angle(SO3ST &s,
                                  double ax, double ay, double az, double angle)
{
    // C++: state->setAxisAngle(ax, ay, az, angle);
    // Sets the quaternion from an axis-angle representation.
    // (ax, ay, az) is the rotation axis (need not be normalised).
    // angle is in radians.
    // Internally: x=ax*sin(a/2), y=ay*sin(a/2), z=az*sin(a/2), w=cos(a/2)
    s.setAxisAngle(ax, ay, az, angle);
}

static void so3st_set_identity(SO3ST &s)
{
    // C++: state->setIdentity();
    // Sets (x, y, z, w) = (0, 0, 0, 1) — the identity rotation.
    // Represents "no rotation at all".
    s.setIdentity();
}

static std::string so3st_repr(const SO3ST &s)
{
    return "<SO3State x=" + std::to_string(s.x) +
           " y=" + std::to_string(s.y) +
           " z=" + std::to_string(s.z) +
           " w=" + std::to_string(s.w) + ">";
}

// -----------------------------------------------------------------------------
// SO3StateSampler helpers
// -----------------------------------------------------------------------------

static void so3samp_sample_uniform(SO3StateSampler &s, SO3ST *st)
{
    // C++: sampler->sampleUniform(state);
    // Samples a uniformly random unit quaternion.
    // Uses the Shoemake method (uniform distribution on S^3).
    s.sampleUniform(static_cast<State *>(st));
}

static void so3samp_sample_uniform_near(SO3StateSampler &s, SO3ST *st,
                                         const SO3ST *near, double distance)
{
    // C++: sampler->sampleUniformNear(state, near, distance);
    // Samples a quaternion uniformly within angular 'distance' of 'near'.
    // Internally: samples a 3-vector from a 3D ball in tangent space,
    // wraps it onto S^3, then pre-multiplies by 'near'.
    s.sampleUniformNear(static_cast<State *>(st),
                        static_cast<const State *>(near),
                        distance);
}

static void so3samp_sample_gaussian(SO3StateSampler &s, SO3ST *st,
                                     const SO3ST *mean, double stddev)
{
    // C++: sampler->sampleGaussian(state, mean, stdDev);
    // Samples a quaternion from a Gaussian distribution centred at 'mean'.
    // Internally: samples a 3-vector from a 3D Gaussian in tangent space,
    // wraps it onto S^3, then pre-multiplies by 'mean'.
    s.sampleGaussian(static_cast<State *>(st),
                     static_cast<const State *>(mean),
                     stddev);
}

// -----------------------------------------------------------------------------
// SO3StateSpace helpers
// -----------------------------------------------------------------------------

static double so3ss_norm(const SO3StateSpace &s, const SO3ST *st)
{
    // C++: space->norm(state)
    // Returns the norm of the quaternion: sqrt(x^2 + y^2 + z^2 + w^2).
    // For a valid unit quaternion this should always be 1.0.
    // Use to check if a quaternion has been correctly normalised.
    return s.norm(st);
}

static unsigned int so3ss_get_dimension(const SO3StateSpace &s)
{
    // C++: space->getDimension()
    // Returns 3. SO(3) has 3 degrees of freedom (3D rotation).
    // Even though a quaternion has 4 components, one degree of freedom
    // is removed by the unit-norm constraint (x^2+y^2+z^2+w^2=1).
    return s.getDimension();
}

static double so3ss_get_maximum_extent(const SO3StateSpace &s)
{
    // C++: space->getMaximumExtent()
    // Returns π/2 (≈ 1.5708).
    // The maximum distance between any two SO(3) states is π/2.
    // Note: this is HALF of what you might expect because q and -q
    // are the same rotation, so the effective maximum arc is π/2.
    return s.getMaximumExtent();
}

static double so3ss_get_measure(const SO3StateSpace &s)
{
    // C++: space->getMeasure()
    // Returns the measure (volume) of SO(3).
    // = π^2 (≈ 9.8696)
    // This is the volume of the 3-sphere S^3 divided by 2 (due to double cover).
    return s.getMeasure();
}

static void so3ss_enforce_bounds(const SO3StateSpace &s, SO3ST *st)
{
    // C++: space->enforceBounds(state);
    // Normalises the quaternion to unit length.
    // If you manually set x,y,z,w to non-unit values, call this
    // to project back onto the unit sphere S^3.
    // Example: if you set x=1,y=1,z=1,w=1 (norm=2), after enforceBounds
    //          you get x=0.5, y=0.5, z=0.5, w=0.5 (norm=1).
    s.enforceBounds(static_cast<State *>(st));
}

static bool so3ss_satisfies_bounds(const SO3StateSpace &s, const SO3ST *st)
{
    // C++: space->satisfiesBounds(state)
    // Returns True if the quaternion has unit norm (within tolerance).
    // A valid SO(3) state MUST be a unit quaternion.
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void so3ss_copy_state(const SO3StateSpace &s, SO3ST *dst, const SO3ST *src)
{
    // C++: space->copyState(destination, source);
    // Copies all 4 quaternion components (x, y, z, w) from src to dst.
    s.copyState(static_cast<State *>(dst),
                static_cast<const State *>(src));
}

static SO3ST *so3ss_clone_state(const SO3StateSpace &s, const SO3ST *src)
{
    // C++: space->cloneState(src)->as<SO3StateSpace::StateType>()
    // Allocates a new state and copies src into it.
    // Must be freed with freeState().
    return s.cloneState(static_cast<const State *>(src))->as<SO3ST>();
}

static unsigned int so3ss_get_serialization_length(const SO3StateSpace &s)
{
    // C++: space->getSerializationLength()
    // Returns 4 * sizeof(double) = 32 bytes.
    // The quaternion has 4 doubles: x, y, z, w.
    return s.getSerializationLength();
}

static double so3ss_distance(const SO3StateSpace &s,
                              const SO3ST *s1, const SO3ST *s2)
{
    // C++: space->distance(state1, state2)
    // Returns the angular distance between two orientations.
    // = acos(|q1 · q2|) where · is the quaternion dot product.
    // The absolute value handles the q/-q double cover.
    // Range: [0, π/2].
    // Example: distance(identity, 90° rotation) = π/4
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool so3ss_equal_states(const SO3StateSpace &s,
                                const SO3ST *s1, const SO3ST *s2)
{
    // C++: space->equalStates(state1, state2)
    // Returns True if both quaternions have exactly the same x,y,z,w.
    // Note: q and -q are NOT equalStates even though they are the same rotation.
    // Normalise both with enforceBounds first for robust comparison.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void so3ss_interpolate(const SO3StateSpace &s,
                               const SO3ST *from, const SO3ST *to,
                               double t, SO3ST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // Interpolates between two orientations using SLERP
    // (Spherical Linear Interpolation).
    // t=0 → result == from
    // t=1 → result == to
    // t=0.5 → halfway between orientations (constant angular velocity)
    // SLERP guarantees: constant angular velocity, shortest path on S^3,
    // result is always a unit quaternion.
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}

static SO3ST *so3ss_alloc_state(SO3StateSpace &s)
{
    // C++: space->allocState()->as<SO3StateSpace::StateType>()
    // Allocates a new SO(3) state on the heap.
    // The state is uninitialised — call setIdentity() or set x,y,z,w explicitly.
    // Always call freeState() when done.
    return s.allocState()->as<SO3ST>();
}

static void so3ss_free_state(SO3StateSpace &s, SO3ST *st)
{
    // C++: space->freeState(state);
    // Frees memory allocated by allocState().
    s.freeState(static_cast<State *>(st));
}

static StateSamplerPtr so3ss_alloc_default_sampler(const SO3StateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    // Returns a SO3StateSampler that generates random unit quaternions.
    return s.allocDefaultStateSampler();
}

static StateSamplerPtr so3ss_alloc_state_sampler(const SO3StateSpace &s)
{
    // C++: space->allocStateSampler()
    // Returns the custom sampler if set, else default SO3StateSampler.
    return s.allocStateSampler();
}

static std::string so3ss_print_state(const SO3StateSpace &s, const SO3ST *st)
{
    // C++: space->printState(state, std::cout);
    // Returns a string showing the quaternion components.
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string so3ss_print_settings(const SO3StateSpace &s)
{
    // C++: space->printSettings(std::cout);
    // Returns a string describing the space: name, type, dimension, etc.
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void so3ss_register_projections(SO3StateSpace &s)
{
    // C++: space->registerProjections();
    // Registers the default projection for SO(3).
    // Projects to R^3 using the quaternion vector part (x, y, z).
    // Called automatically by setup().
    s.registerProjections();
}

static void so3ss_setup(SO3StateSpace &s)
{
    // C++: space->setup();
    // Finalises the space. Must be called before copyToReals/copyFromReals.
    // Called automatically by SpaceInformation.
    s.setup();
}

static std::vector<double> so3ss_copy_to_reals(const SO3StateSpace &s, const SO3ST *st)
{
    // C++: space->copyToReals(reals, source);
    // Returns the quaternion as a 4-element list: [x, y, z, w].
    // setup() must be called first.
    const_cast<SO3StateSpace &>(s).computeLocations();
    std::vector<double> reals;
    s.copyToReals(reals, static_cast<const State *>(st));
    return reals;
}

static void so3ss_copy_from_reals(const SO3StateSpace &s,
                                   SO3ST *dst,
                                   const std::vector<double> &reals)
{
    // C++: space->copyFromReals(destination, reals);
    // Sets the quaternion from a 4-element list: [x, y, z, w].
    // setup() must be called first.
    const_cast<SO3StateSpace &>(s).computeLocations();
    s.copyFromReals(static_cast<State *>(dst), reals);
}

static std::string so3ss_repr(const SO3StateSpace &s)
{
    return "<SO3StateSpace name=" + s.getName() + ">";
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_so3_state_space(py::module_ &m)
{
    // -------------------------------------------------------------------------
    // SO3StateSpace::StateType
    // -------------------------------------------------------------------------
    py::class_<SO3ST>(m, "SO3State",
        "A single state in SO(3) — a unit quaternion (x, y, z, w).\n\n"
        "C++ type:   ompl::base::SO3StateSpace::StateType\n"
        "C++ header: ompl/base/spaces/SO3StateSpace.h\n\n"
        "WHAT IS A UNIT QUATERNION?\n"
        "A quaternion q = (x, y, z, w) represents a 3D rotation:\n"
        "  - (x, y, z): vector part = axis * sin(angle/2)\n"
        "  - w:         scalar part = cos(angle/2)\n"
        "  - constraint: x^2 + y^2 + z^2 + w^2 = 1 (unit norm)\n\n"
        "Identity rotation (no rotation): x=0, y=0, z=0, w=1\n"
        "180° around Z axis:              x=0, y=0, z=1, w=0\n"
        "90° around Y axis:               x=0, y=sin(π/4), z=0, w=cos(π/4)\n\n"
        "IMPORTANT: q and -q represent the SAME rotation.\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocState() to get one.")

        .def_property("x", &so3st_get_x, &so3st_set_x,
            "X component of the quaternion vector part.\n"
            "C++: state->x\n"
            "= axis.x * sin(angle/2)")

        .def_property("y", &so3st_get_y, &so3st_set_y,
            "Y component of the quaternion vector part.\n"
            "C++: state->y\n"
            "= axis.y * sin(angle/2)")

        .def_property("z", &so3st_get_z, &so3st_set_z,
            "Z component of the quaternion vector part.\n"
            "C++: state->z\n"
            "= axis.z * sin(angle/2)")

        .def_property("w", &so3st_get_w, &so3st_set_w,
            "Scalar component of the quaternion.\n"
            "C++: state->w\n"
            "= cos(angle/2)\n"
            "For identity rotation: w=1. For 180° rotation: w=0.")

        .def("setAxisAngle", &so3st_set_axis_angle,
            py::arg("ax"), py::arg("ay"), py::arg("az"), py::arg("angle"),
            "Set the quaternion from axis-angle representation.\n\n"
            "C++: state->setAxisAngle(ax, ay, az, angle);\n\n"
            "Parameters:\n"
            "  ax, ay, az — rotation axis (does not need to be unit length)\n"
            "  angle      — rotation amount in RADIANS\n\n"
            "Internally normalises the axis and computes:\n"
            "  x = ax_norm * sin(angle/2)\n"
            "  y = ay_norm * sin(angle/2)\n"
            "  z = az_norm * sin(angle/2)\n"
            "  w = cos(angle/2)\n\n"
            "Examples:\n"
            "  90° around Z: setAxisAngle(0, 0, 1, π/2)\n"
            "  45° around Y: setAxisAngle(0, 1, 0, π/4)")

        .def("setIdentity", &so3st_set_identity,
            "Set to identity rotation — no rotation at all.\n\n"
            "C++: state->setIdentity();\n\n"
            "Sets (x, y, z, w) = (0, 0, 0, 1).\n"
            "This represents a robot facing its default orientation.")

        .def("__repr__", &so3st_repr);

    // -------------------------------------------------------------------------
    // SO3StateSampler
    // -------------------------------------------------------------------------
    py::class_<SO3StateSampler, StateSampler,
               std::shared_ptr<SO3StateSampler>>(m, "SO3StateSampler",
        "Sampler for SO(3) — generates random unit quaternions.\n\n"
        "C++ class:  ompl::base::SO3StateSampler\n"
        "C++ header: ompl/base/spaces/SO3StateSpace.h\n\n"
        "All three sampling methods use the tangent space approach:\n"
        "sample a 3-vector in R^3 tangent space, then map it to S^3.\n"
        "This ensures uniform/Gaussian distribution on the 3-sphere.\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocDefaultStateSampler() to get one.")

        .def("sampleUniform", &so3samp_sample_uniform, py::arg("state"),
            "Sample a uniformly random rotation (unit quaternion).\n\n"
            "C++: sampler->sampleUniform(state);\n\n"
            "Uses the Shoemake method for uniform distribution on S^3.\n"
            "Every possible 3D rotation is equally likely.")

        .def("sampleUniformNear", &so3samp_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample a rotation uniformly within 'distance' radians of 'near'.\n\n"
            "C++: sampler->sampleUniformNear(state, near, distance);\n\n"
            "Samples a 3-vector from a 3D ball in tangent space (radius=distance),\n"
            "maps it to S^3, then pre-multiplies by 'near' quaternion.\n"
            "'distance' is in radians.")

        .def("sampleGaussian", &so3samp_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample a rotation from a Gaussian centred at 'mean'.\n\n"
            "C++: sampler->sampleGaussian(state, mean, stdDev);\n\n"
            "Samples a 3-vector from 3D Gaussian in tangent space,\n"
            "maps to S^3, pre-multiplies by 'mean' quaternion.\n"
            "'stdDev' is in radians.");

    // -------------------------------------------------------------------------
    // SO3StateSpace
    // -------------------------------------------------------------------------
    py::class_<SO3StateSpace, StateSpace,
               std::shared_ptr<SO3StateSpace>>(m, "SO3StateSpace",
        "State space representing SO(3) — all possible 3D rotations.\n\n"
        "C++ header: ompl/base/spaces/SO3StateSpace.h\n"
        "C++ class:  ompl::base::SO3StateSpace\n"
        "Inherits from: StateSpace\n\n"
        "INTERNAL REPRESENTATION: unit quaternions (x, y, z, w)\n"
        "  - 4 components, constrained to unit sphere S^3\n"
        "  - 3 degrees of freedom (since |q|=1 removes one)\n\n"
        "WHY QUATERNIONS?\n"
        "  - No gimbal lock (unlike Euler angles)\n"
        "  - Smooth interpolation via SLERP\n"
        "  - Well-defined distance metric\n\n"
        "DISTANCE: acos(|q1·q2|) ∈ [0, π/2]\n"
        "INTERPOLATION: SLERP (constant angular velocity)\n"
        "MAX EXTENT: π/2 (due to q/-q double cover)\n\n"
        "USE FOR: 3D robot orientation, drone attitude, camera pose,\n"
        "         any joint that rotates freely in 3D.")

        .def(py::init<>(),
            "Construct an SO(3) state space.\n\n"
            "C++: auto space = std::make_shared<SO3StateSpace>();\n\n"
            "No bounds needed — SO(3) is bounded by its spherical topology.\n"
            "Call setup() before using copyToReals/copyFromReals.")

        .def("norm", &so3ss_norm, py::arg("state"),
            "Return the norm of the quaternion: sqrt(x^2 + y^2 + z^2 + w^2).\n\n"
            "C++: space->norm(state)\n\n"
            "A valid SO(3) state has norm == 1.0 (unit quaternion).\n"
            "Use this to verify a state is valid before planning.\n"
            "If norm != 1.0, call enforceBounds() to normalise.")

        .def("getDimension", &so3ss_get_dimension,
            "Return the dimension of SO(3): always 3.\n\n"
            "C++: space->getDimension()\n\n"
            "SO(3) has 3 degrees of freedom (roll, pitch, yaw).\n"
            "Even though the quaternion has 4 components, the unit-norm\n"
            "constraint reduces the dimensionality to 3.")

        .def("getMaximumExtent", &so3ss_get_maximum_extent,
            "Return the maximum distance between any two SO(3) states.\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "Returns π/2 (≈ 1.5708).\n"
            "This is HALF of π because q and -q are the same rotation,\n"
            "so the effective range of the distance metric is [0, π/2].")

        .def("getMeasure", &so3ss_get_measure,
            "Return the measure (volume) of SO(3).\n\n"
            "C++: space->getMeasure()\n\n"
            "Returns π^2 (≈ 9.8696).\n"
            "This is the Haar measure of SO(3) — the 'volume' of all\n"
            "possible 3D rotations.")

        .def("enforceBounds", &so3ss_enforce_bounds, py::arg("state"),
            "Normalise the quaternion to unit length. Modifies in place.\n\n"
            "C++: space->enforceBounds(state);\n\n"
            "If you set x,y,z,w manually to non-unit values, call this\n"
            "to project back onto the unit sphere S^3.\n"
            "Formula: q = q / |q|\n\n"
            "Always call after manually setting quaternion components,\n"
            "or use setAxisAngle()/setIdentity() which handle this automatically.")

        .def("satisfiesBounds", &so3ss_satisfies_bounds, py::arg("state"),
            "Return True if the quaternion has unit norm (within tolerance).\n\n"
            "C++: space->satisfiesBounds(state)\n\n"
            "Checks: |x^2 + y^2 + z^2 + w^2 - 1| < tolerance\n"
            "A valid SO(3) state MUST be a unit quaternion.\n"
            "Does NOT modify the state — use enforceBounds() to fix it.")

        .def("copyState", &so3ss_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy all 4 quaternion components from source to destination.\n\n"
            "C++: space->copyState(destination, source);\n\n"
            "Memory of source and destination must not overlap.\n"
            "After: dst.x==src.x, dst.y==src.y, dst.z==src.z, dst.w==src.w")

        .def("cloneState", &so3ss_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n\n"
            "C++: space->cloneState(src)->as<SO3StateSpace::StateType>()\n\n"
            "Must be freed with freeState().")

        .def("copyToReals", &so3ss_copy_to_reals, py::arg("source"),
            "Return quaternion components as a 4-element list [x, y, z, w].\n\n"
            "C++: space->copyToReals(reals, source);\n\n"
            "Requires setup() to have been called first.")

        .def("copyFromReals", &so3ss_copy_from_reals,
            py::arg("destination"), py::arg("reals"),
            "Set quaternion from a 4-element list [x, y, z, w].\n\n"
            "C++: space->copyFromReals(destination, reals);\n\n"
            "reals must have exactly 4 elements.\n"
            "Does NOT automatically normalise — call enforceBounds() if needed.\n"
            "Requires setup() to have been called first.")

        .def("getSerializationLength", &so3ss_get_serialization_length,
            "Return bytes needed to serialise one SO(3) state.\n\n"
            "C++: space->getSerializationLength()\n\n"
            "Returns 32 = 4 * sizeof(double) (four quaternion components).")

        .def("distance", &so3ss_distance, py::arg("state1"), py::arg("state2"),
            "Return the angular distance between two 3D orientations.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "Formula: acos(|q1·q2|)  where · is quaternion dot product.\n"
            "The absolute value handles the q/-q double cover.\n"
            "Range: [0, π/2].\n\n"
            "Examples:\n"
            "  distance(identity, identity)    = 0\n"
            "  distance(identity, 90° around Z) = π/4 ≈ 0.785\n"
            "  distance(identity, 180° around Z) = π/2 ≈ 1.571")

        .def("equalStates", &so3ss_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if both quaternions have exactly the same x,y,z,w.\n\n"
            "C++: space->equalStates(state1, state2)\n\n"
            "Exact floating-point comparison.\n"
            "Note: q and -q represent the same rotation but are NOT equalStates.\n"
            "Use distance() < epsilon for geometric equality check.")

        .def("interpolate", &so3ss_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Interpolate between two orientations using SLERP.\n\n"
            "C++: space->interpolate(from, to, t, result);\n\n"
            "SLERP = Spherical Linear Interpolation.\n"
            "t=0 → from,  t=1 → to,  t=0.5 → halfway.\n\n"
            "Properties of SLERP:\n"
            "  - Constant angular velocity (no speed-up/slow-down)\n"
            "  - Shortest path on S^3\n"
            "  - Result is always a unit quaternion\n"
            "  - Smooth (no gimbal lock)\n\n"
            "'result' must be a pre-allocated state.")

        .def("allocState", &so3ss_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new SO(3) state. Memory managed by the space.\n\n"
            "C++: space->allocState()->as<SO3StateSpace::StateType>()\n\n"
            "State is uninitialised. Call setIdentity() or setAxisAngle() first.\n"
            "Always call freeState() when done.")

        .def("freeState", &so3ss_free_state, py::arg("state"),
            "Free a state previously allocated by allocState().\n\n"
            "C++: space->freeState(state);")

        .def("allocDefaultStateSampler", &so3ss_alloc_default_sampler,
            "Allocate the default SO(3) sampler.\n\n"
            "C++: space->allocDefaultStateSampler()\n\n"
            "Returns a SO3StateSampler for generating random orientations.")

        .def("allocStateSampler", &so3ss_alloc_state_sampler,
            "Allocate a sampler (custom if set, else default).\n\n"
            "C++: space->allocStateSampler()")

        .def("printState", &so3ss_print_state, py::arg("state"),
            "Return a string showing the quaternion components.\n\n"
            "C++: space->printState(state, std::cout);")

        .def("printSettings", &so3ss_print_settings,
            "Return a string describing this space's configuration.\n\n"
            "C++: space->printSettings(std::cout);")

        .def("registerProjections", &so3ss_register_projections,
            "Register the default projection for SO(3).\n\n"
            "C++: space->registerProjections();\n\n"
            "Projects to R^3 using the (x, y, z) quaternion components.\n"
            "Called automatically by setup().")

        .def("setup", &so3ss_setup,
            "Finalise the space. Call before copyToReals/copyFromReals.\n\n"
            "C++: space->setup();\n\n"
            "Called automatically by SpaceInformation.")

        .def("getName", &SO3StateSpace::getName,
            "Return the name of this space.\n"
            "C++: space->getName()")

        .def("setName", &SO3StateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n"
            "C++: space->setName(name);")

        .def("__repr__", &so3ss_repr);
}