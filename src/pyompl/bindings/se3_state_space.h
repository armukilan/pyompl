#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience aliases
using SE3ST  = SE3StateSpace::StateType;
using SO3ST  = SO3StateSpace::StateType;

// =============================================================================
// WHAT IS SE(3)?
// --------------
// SE(3) = Special Euclidean Group in 3 dimensions.
// It represents the COMPLETE POSE of a rigid body in 3D space:
//   x, y, z   — 3D position      (from RealVectorStateSpace(3))
//   rotation  — 3D orientation   (from SO3StateSpace, stored as quaternion)
//
// Total degrees of freedom: 6 (3 position + 3 rotation)
//
// INTERNALLY SE(3) is a COMPOUND STATE SPACE:
//   Subspace 0: RealVectorStateSpace(3)  weight=1.0  → (x, y, z)
//   Subspace 1: SO3StateSpace()          weight=1.0  → quaternion (qx,qy,qz,qw)
//
// COMPOUND DISTANCE:
//   d = 1.0 × euclidean(p1, p2)  +  1.0 × acos(|q1·q2|)
//
// COMPOUND INTERPOLATION:
//   position: linear         (x,y,z interpolated linearly)
//   rotation: SLERP          (quaternion spherical linear interpolation)
//
// USE FOR:
//   - 6-DOF robot arm end-effector pose
//   - Drone position + attitude (3D)
//   - Any rigid body moving freely in 3D
//   - Object manipulation planning
//
// DIFFERENCE FROM SE(2):
//   SE(2) = x, y, yaw          (2D plane, 3 DOF)
//   SE(3) = x, y, z, quaternion (3D space, 6 DOF)
//
// C++ header: ompl/base/spaces/SE3StateSpace.h
// C++ class:  ompl::base::SE3StateSpace
// Inherits:   CompoundStateSpace → StateSpace
// =============================================================================

// -----------------------------------------------------------------------------
// SE3StateSpace::StateType helpers
// -----------------------------------------------------------------------------

static double se3st_get_x(const SE3ST &s) { return s.getX(); }
static double se3st_get_y(const SE3ST &s) { return s.getY(); }
static double se3st_get_z(const SE3ST &s) { return s.getZ(); }

static void se3st_set_x(SE3ST &s, double v) { s.setX(v); }
static void se3st_set_y(SE3ST &s, double v) { s.setY(v); }
static void se3st_set_z(SE3ST &s, double v) { s.setZ(v); }

static void se3st_set_xyz(SE3ST &s, double x, double y, double z)
{
    // C++: state->setXYZ(x, y, z);
    // Sets all three position components in one call.
    s.setXYZ(x, y, z);
}

static SO3ST &se3st_get_rotation(SE3ST &s)
{
    // C++: state->rotation()
    // Returns a reference to the SO3StateSpace::StateType that stores
    // the orientation quaternion (qx, qy, qz, qw).
    // You can read and modify it directly.
    return s.rotation();
}

static std::string se3st_repr(const SE3ST &s)
{
    const SO3ST &r = s.rotation();
    return "<SE3State x=" + std::to_string(s.getX()) +
           " y=" + std::to_string(s.getY()) +
           " z=" + std::to_string(s.getZ()) +
           " qx=" + std::to_string(r.x) +
           " qy=" + std::to_string(r.y) +
           " qz=" + std::to_string(r.z) +
           " qw=" + std::to_string(r.w) + ">";
}

// -----------------------------------------------------------------------------
// SE3StateSpace helpers
// -----------------------------------------------------------------------------

static void se3ss_set_bounds(SE3StateSpace &s, const RealVectorBounds &b)
{
    // C++: space->setBounds(bounds);
    // Sets the position (x, y, z) bounds.
    // ONLY applies to the RealVectorStateSpace(3) subspace.
    // The rotation subspace (SO3) has no bounds — unit quaternion constraint
    // is its own "bound".
    s.setBounds(b);
}

static const RealVectorBounds &se3ss_get_bounds(const SE3StateSpace &s)
{
    // C++: space->getBounds()
    // Returns the bounds of the position subspace.
    return s.getBounds();
}

static SE3ST *se3ss_alloc_state(SE3StateSpace &s)
{
    // C++: space->allocState()->as<SE3StateSpace::StateType>()
    // Allocates a new SE(3) state. Memory is managed by the space.
    // The state is uninitialised — set x,y,z and rotation before using.
    // Always call freeState() when done.
    return s.allocState()->as<SE3ST>();
}

static void se3ss_free_state(SE3StateSpace &s, SE3ST *st)
{
    // C++: space->freeState(state);
    // Frees memory allocated by allocState().
    s.freeState(static_cast<State *>(st));
}

static SE3ST *se3ss_clone_state(const SE3StateSpace &s, const SE3ST *src)
{
    // C++: space->cloneState(src)->as<SE3StateSpace::StateType>()
    // Allocates a new state and copies src. Must be freed with freeState().
    return s.cloneState(static_cast<const State *>(src))->as<SE3ST>();
}

static void se3ss_copy_state(const SE3StateSpace &s, SE3ST *dst, const SE3ST *src)
{
    // C++: space->copyState(destination, source);
    // Copies all 6 components (x,y,z and quaternion) from src to dst.
    s.copyState(static_cast<State *>(dst),
                static_cast<const State *>(src));
}

static double se3ss_distance(const SE3StateSpace &s,
                              const SE3ST *s1, const SE3ST *s2)
{
    // C++: space->distance(state1, state2)
    // Weighted compound distance:
    //   d = 1.0 × euclidean((x1,y1,z1),(x2,y2,z2))
    //     + 1.0 × acos(|q1·q2|)
    // Both subspaces have weight 1.0 in SE(3).
    // Position in world units, rotation in radians [0, π/2].
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool se3ss_equal_states(const SE3StateSpace &s,
                                const SE3ST *s1, const SE3ST *s2)
{
    // C++: space->equalStates(state1, state2)
    // Returns True if x, y, z AND all quaternion components are exactly equal.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void se3ss_interpolate(const SE3StateSpace &s,
                               const SE3ST *from, const SE3ST *to,
                               double t, SE3ST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // Interpolates position LINEARLY and rotation via SLERP.
    //   result.x = from.x + t*(to.x - from.x)
    //   result.y = from.y + t*(to.y - from.y)
    //   result.z = from.z + t*(to.z - from.z)
    //   result.rotation = SLERP(from.rotation, to.rotation, t)
    // t=0 → from, t=1 → to.
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}

static bool se3ss_satisfies_bounds(const SE3StateSpace &s, const SE3ST *st)
{
    // C++: space->satisfiesBounds(state)
    // Returns True if x,y,z are within position bounds AND
    // the quaternion has unit norm.
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void se3ss_enforce_bounds(const SE3StateSpace &s, SE3ST *st)
{
    // C++: space->enforceBounds(state);
    // Clamps x,y,z to position bounds AND normalises the quaternion.
    s.enforceBounds(static_cast<State *>(st));
}

static unsigned int se3ss_get_dimension(const SE3StateSpace &s)
{
    // C++: space->getDimension()
    // Returns 6: 3 for position (x,y,z) + 3 for rotation (SO(3) has dim 3).
    return s.getDimension();
}

static double se3ss_get_maximum_extent(const SE3StateSpace &s)
{
    // C++: space->getMaximumExtent()
    // Returns: 1.0 × max_position_dist + 1.0 × π/2
    // Where max_position_dist is the diagonal of the position bounding box.
    return s.getMaximumExtent();
}

static double se3ss_get_measure(const SE3StateSpace &s)
{
    // C++: space->getMeasure()
    // Returns the weighted volume of SE(3):
    //   volume_of_position_box × π^2
    return s.getMeasure();
}

static unsigned int se3ss_get_serialization_length(const SE3StateSpace &s)
{
    // C++: space->getSerializationLength()
    // Returns 7 * sizeof(double) = 56 bytes (x,y,z + qx,qy,qz,qw).
    return s.getSerializationLength();
}

static std::vector<double> se3ss_copy_to_reals(const SE3StateSpace &s, const SE3ST *st)
{
    // C++: space->copyToReals(reals, source);
    // Returns [x, y, z, qx, qy, qz, qw] as a 7-element Python list.
    // setup() must be called first.
    const_cast<SE3StateSpace &>(s).computeLocations();
    std::vector<double> reals;
    s.copyToReals(reals, static_cast<const State *>(st));
    return reals;
}

static void se3ss_copy_from_reals(const SE3StateSpace &s,
                                   SE3ST *dst,
                                   const std::vector<double> &reals)
{
    // C++: space->copyFromReals(destination, reals);
    // Sets state from a 7-element list [x, y, z, qx, qy, qz, qw].
    // setup() must be called first.
    const_cast<SE3StateSpace &>(s).computeLocations();
    s.copyFromReals(static_cast<State *>(dst), reals);
}

static StateSamplerPtr se3ss_alloc_default_sampler(const SE3StateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    // Returns a CompoundStateSampler that independently samples:
    //   (x,y,z) uniformly from position bounds
    //   rotation uniformly from SO(3) (uniform random unit quaternion)
    return s.allocDefaultStateSampler();
}

static StateSamplerPtr se3ss_alloc_state_sampler(const SE3StateSpace &s)
{
    // C++: space->allocStateSampler()
    return s.allocStateSampler();
}

static std::string se3ss_print_state(const SE3StateSpace &s, const SE3ST *st)
{
    // C++: space->printState(state, std::cout);
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string se3ss_print_settings(const SE3StateSpace &s)
{
    // C++: space->printSettings(std::cout);
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void se3ss_register_projections(SE3StateSpace &s)
{
    // C++: space->registerProjections();
    // Registers the default SE(3) projection: maps to R^3 using (x,y,z).
    // Called automatically by setup().
    s.registerProjections();
}

static void se3ss_setup(SE3StateSpace &s)
{
    // C++: space->setup();
    // Finalises the space. Must be called after setBounds().
    // Called automatically by SpaceInformation.
    s.setup();
}

// Subspace access
static unsigned int se3ss_get_subspace_count(const SE3StateSpace &s)
{
    // C++: space->getSubspaceCount()
    // Returns 2: subspace 0 = RealVectorStateSpace(3), subspace 1 = SO3StateSpace.
    return s.getSubspaceCount();
}

static double se3ss_get_subspace_weight(const SE3StateSpace &s, unsigned int i)
{
    // C++: space->getSubspaceWeight(i)
    // Both subspaces have weight 1.0 in SE(3).
    return s.getSubspaceWeight(i);
}

static bool se3ss_is_locked(const SE3StateSpace &s)
{
    // C++: space->isLocked()
    // Always True — SE3StateSpace locks itself in the constructor.
    return s.isLocked();
}

// Direct sampling helpers (avoids cast issues with StateSampler base class)
static void se3ss_sample_uniform(SE3StateSpace &s, SE3ST *st)
{
    // C++: space->allocDefaultStateSampler()->sampleUniform(state);
    // Samples (x,y,z) uniformly from bounds and rotation uniformly from SO(3).
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniform(static_cast<State *>(st));
}

static void se3ss_sample_uniform_near(SE3StateSpace &s, SE3ST *st,
                                       const SE3ST *near, double dist)
{
    // C++: space->allocDefaultStateSampler()->sampleUniformNear(state, near, dist);
    // Samples within 'dist' of 'near' in the compound metric.
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniformNear(static_cast<State *>(st),
                               static_cast<const State *>(near), dist);
}

static void se3ss_sample_gaussian(SE3StateSpace &s, SE3ST *st,
                                   const SE3ST *mean, double stddev)
{
    // C++: space->allocDefaultStateSampler()->sampleGaussian(state, mean, stdDev);
    // Samples from Gaussian centred at 'mean'.
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleGaussian(static_cast<State *>(st),
                            static_cast<const State *>(mean), stddev);
}

static std::string se3ss_repr(const SE3StateSpace &s)
{
    return "<SE3StateSpace name=" + s.getName() + ">";
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_se3_state_space(py::module_ &m)
{
    // -------------------------------------------------------------------------
    // SE3StateSpace::StateType
    // -------------------------------------------------------------------------
    py::class_<SE3ST>(m, "SE3State",
        "A state in SE(3): 3D position (x,y,z) + 3D orientation (quaternion).\n\n"
        "C++ type:   ompl::base::SE3StateSpace::StateType\n"
        "C++ header: ompl/base/spaces/SE3StateSpace.h\n"
        "Inherits:   CompoundStateSpace::StateType\n\n"
        "WHAT IS AN SE(3) STATE?\n"
        "Complete 6-DOF pose of a rigid body in 3D:\n"
        "  x, y, z    — 3D position\n"
        "  rotation   — 3D orientation as unit quaternion (qx, qy, qz, qw)\n\n"
        "INTERNAL STRUCTURE:\n"
        "  substate[0] = RealVectorStateSpace::StateType → (x, y, z)\n"
        "  substate[1] = SO3StateSpace::StateType        → quaternion\n\n"
        "Do NOT construct directly. Use space.allocState().")

        .def_property("x", &se3st_get_x, &se3st_set_x,
            "X position component.\n"
            "C++: state->getX() / state->setX(v)\n"
            "Internally: state->as<RealVectorStateSpace::StateType>(0)->values[0]")

        .def_property("y", &se3st_get_y, &se3st_set_y,
            "Y position component.\n"
            "C++: state->getY() / state->setY(v)")

        .def_property("z", &se3st_get_z, &se3st_set_z,
            "Z position component.\n"
            "C++: state->getZ() / state->setZ(v)")

        .def("getX",   &se3st_get_x,   "Get X position. C++: state->getX()")
        .def("getY",   &se3st_get_y,   "Get Y position. C++: state->getY()")
        .def("getZ",   &se3st_get_z,   "Get Z position. C++: state->getZ()")
        .def("setX",   &se3st_set_x,   py::arg("x"), "Set X position. C++: state->setX(x);")
        .def("setY",   &se3st_set_y,   py::arg("y"), "Set Y position. C++: state->setY(y);")
        .def("setZ",   &se3st_set_z,   py::arg("z"), "Set Z position. C++: state->setZ(z);")

        .def("setXYZ", &se3st_set_xyz,
            py::arg("x"), py::arg("y"), py::arg("z"),
            "Set all three position components at once.\n\n"
            "C++: state->setXYZ(x, y, z);\n\n"
            "Equivalent to setX(x); setY(y); setZ(z);")

        .def("rotation", &se3st_get_rotation,
            py::return_value_policy::reference_internal,
            "Return a reference to the SO3State (quaternion) component.\n\n"
            "C++: state->rotation()  (returns SO3StateSpace::StateType&)\n\n"
            "The returned object has .x .y .z .w properties and\n"
            ".setAxisAngle() and .setIdentity() methods.\n\n"
            "Example:\n"
            "  state.rotation().setIdentity()           # no rotation\n"
            "  state.rotation().setAxisAngle(0,0,1,PI)  # 180° around Z\n"
            "  qx = state.rotation().x                  # read quaternion x")

        .def("__repr__", &se3st_repr);

    // -------------------------------------------------------------------------
    // SE3StateSpace
    // -------------------------------------------------------------------------
    py::class_<SE3StateSpace, CompoundStateSpace,
               std::shared_ptr<SE3StateSpace>>(m, "SE3StateSpace",
        "State space representing SE(3) — 3D position + 3D orientation.\n\n"
        "C++ header: ompl/base/spaces/SE3StateSpace.h\n"
        "C++ class:  ompl::base::SE3StateSpace\n"
        "Inherits:   CompoundStateSpace → StateSpace\n\n"
        "SE(3) = Special Euclidean Group in 3D.\n"
        "Represents all possible poses of a rigid body in 3D space.\n\n"
        "INTERNAL STRUCTURE (CompoundStateSpace):\n"
        "  Subspace 0: RealVectorStateSpace(3)  weight=1.0  → (x,y,z)\n"
        "  Subspace 1: SO3StateSpace()           weight=1.0  → quaternion\n\n"
        "DISTANCE FORMULA:\n"
        "  d = 1.0 × euclidean(p1,p2) + 1.0 × acos(|q1·q2|)\n\n"
        "INTERPOLATION:\n"
        "  Position: linear  |  Rotation: SLERP\n\n"
        "USE FOR: robot end-effector, drone in 3D, object manipulation.\n\n"
        "SETUP REQUIRED:\n"
        "  1. space = SE3StateSpace()\n"
        "  2. bounds = RealVectorBounds(3); ... ; space.setBounds(bounds)\n"
        "  3. space.setup()")

        .def(py::init<>(),
            "Construct an SE(3) state space.\n\n"
            "C++: auto space = std::make_shared<SE3StateSpace>();\n\n"
            "Automatically creates:\n"
            "  - RealVectorStateSpace(3) with weight 1.0\n"
            "  - SO3StateSpace() with weight 1.0\n"
            "You MUST call setBounds() for the position subspace.")

        .def("setBounds", &se3ss_set_bounds, py::arg("bounds"),
            "Set the 3D position (x,y,z) bounds.\n\n"
            "C++: space->setBounds(bounds);\n\n"
            "Sets bounds for the RealVectorStateSpace(3) subspace.\n"
            "MUST be called before using the space.\n"
            "Rotation bounds are NOT needed — SO(3) is self-bounding.\n\n"
            "Example:\n"
            "  bounds = RealVectorBounds(3)\n"
            "  bounds.setLow(-5.0)   # x,y,z >= -5\n"
            "  bounds.setHigh(5.0)   # x,y,z <= 5\n"
            "  space.setBounds(bounds)")

        .def("getBounds", &se3ss_get_bounds,
            py::return_value_policy::reference_internal,
            "Return the current 3D position bounds.\n\n"
            "C++: space->getBounds()")

        .def("getDimension", &se3ss_get_dimension,
            "Return total dimension: 6 (3 position + 3 rotation).\n\n"
            "C++: space->getDimension()\n\n"
            "Even though the quaternion has 4 components, SO(3) has\n"
            "3 DOF (unit norm removes one), so total is 3+3=6.")

        .def("getMaximumExtent", &se3ss_get_maximum_extent,
            "Return the weighted maximum extent of SE(3).\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "= 1.0 × diagonal_of_position_box + 1.0 × π/2")

        .def("getMeasure", &se3ss_get_measure,
            "Return the weighted volume of SE(3).\n\n"
            "C++: space->getMeasure()\n\n"
            "= volume_of_position_box × π^2")

        .def("satisfiesBounds", &se3ss_satisfies_bounds, py::arg("state"),
            "Return True if position is within bounds AND quaternion is unit.\n\n"
            "C++: space->satisfiesBounds(state)")

        .def("enforceBounds", &se3ss_enforce_bounds, py::arg("state"),
            "Clamp x,y,z to bounds and normalise the quaternion.\n\n"
            "C++: space->enforceBounds(state);")

        .def("copyState", &se3ss_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy all 7 components (x,y,z,qx,qy,qz,qw) from source to destination.\n\n"
            "C++: space->copyState(destination, source);")

        .def("cloneState", &se3ss_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n\n"
            "C++: space->cloneState(src)->as<SE3StateSpace::StateType>()\n\n"
            "Must be freed with freeState().")

        .def("copyToReals", &se3ss_copy_to_reals, py::arg("source"),
            "Return [x, y, z, qx, qy, qz, qw] as a 7-element list.\n\n"
            "C++: space->copyToReals(reals, source);\n\n"
            "Requires setup() to have been called first.")

        .def("copyFromReals", &se3ss_copy_from_reals,
            py::arg("destination"), py::arg("reals"),
            "Set state from a 7-element list [x, y, z, qx, qy, qz, qw].\n\n"
            "C++: space->copyFromReals(destination, reals);\n\n"
            "Requires setup() to have been called first.")

        .def("getSerializationLength", &se3ss_get_serialization_length,
            "Return bytes to serialise one SE(3) state.\n\n"
            "C++: space->getSerializationLength()\n\n"
            "Returns 56 = 7 × sizeof(double).")

        .def("distance", &se3ss_distance, py::arg("state1"), py::arg("state2"),
            "Return the weighted compound distance between two SE(3) states.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "Formula:\n"
            "  d = 1.0 × euclidean((x1,y1,z1),(x2,y2,z2))\n"
            "    + 1.0 × acos(|q1·q2|)\n\n"
            "Both subspaces have equal weight 1.0.\n"
            "Note: units are mixed (metres + radians), but planners handle this.")

        .def("equalStates", &se3ss_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if all 7 components are exactly equal.\n\n"
            "C++: space->equalStates(state1, state2)")

        .def("interpolate", &se3ss_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Interpolate between two SE(3) states at fraction t.\n\n"
            "C++: space->interpolate(from, to, t, result);\n\n"
            "t=0 → from,  t=1 → to,  t=0.5 → midpoint.\n\n"
            "Position: linear interpolation of (x,y,z)\n"
            "Rotation: SLERP interpolation of quaternion\n\n"
            "'result' must be pre-allocated.")

        .def("allocState", &se3ss_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new SE(3) state. Memory managed by the space.\n\n"
            "C++: space->allocState()->as<SE3StateSpace::StateType>()\n\n"
            "State is uninitialised. Set x,y,z and rotation before using.\n"
            "Always call freeState() when done.")

        .def("freeState", &se3ss_free_state, py::arg("state"),
            "Free a state previously allocated by allocState().\n\n"
            "C++: space->freeState(state);")

        .def("allocDefaultStateSampler", &se3ss_alloc_default_sampler,
            "Allocate the default SE(3) sampler.\n\n"
            "C++: space->allocDefaultStateSampler()\n\n"
            "Returns a CompoundStateSampler.\n"
            "Use sampleUniform/sampleUniformNear/sampleGaussian instead\n"
            "to avoid type casting issues.")

        .def("allocStateSampler", &se3ss_alloc_state_sampler,
            "Allocate a sampler (custom if set, else default).\n\n"
            "C++: space->allocStateSampler()")

        .def("sampleUniform", &se3ss_sample_uniform, py::arg("state"),
            "Sample a random SE(3) state within the bounds.\n\n"
            "Samples (x,y,z) uniformly from position bounds and\n"
            "rotation uniformly from SO(3).\n"
            "C++: space->allocDefaultStateSampler()->sampleUniform(state);")

        .def("sampleUniformNear", &se3ss_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample a random SE(3) state near 'near' within 'distance'.\n\n"
            "C++: space->allocDefaultStateSampler()->sampleUniformNear(...);")

        .def("sampleGaussian", &se3ss_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample a random SE(3) state from Gaussian centred at 'mean'.\n\n"
            "C++: space->allocDefaultStateSampler()->sampleGaussian(...);")

        .def("printState", &se3ss_print_state, py::arg("state"),
            "Return a string showing position and quaternion.\n\n"
            "C++: space->printState(state, std::cout);")

        .def("printSettings", &se3ss_print_settings,
            "Return a string describing the space settings.\n\n"
            "C++: space->printSettings(std::cout);")

        .def("registerProjections", &se3ss_register_projections,
            "Register the default SE(3) projection.\n\n"
            "C++: space->registerProjections();\n\n"
            "Maps SE(3) states to R^3 using (x,y,z) position.\n"
            "Called automatically by setup().")

        .def("setup", &se3ss_setup,
            "Finalise the space. Call after setBounds().\n\n"
            "C++: space->setup();")

        .def("getSubspaceCount", &se3ss_get_subspace_count,
            "Return the number of subspaces: always 2 for SE(3).\n\n"
            "C++: space->getSubspaceCount()\n\n"
            "Subspace 0: RealVectorStateSpace(3) — position\n"
            "Subspace 1: SO3StateSpace()          — orientation")

        .def("getSubspaceWeight", &se3ss_get_subspace_weight, py::arg("index"),
            "Return the weight of subspace i.\n\n"
            "C++: space->getSubspaceWeight(index)\n\n"
            "Both subspaces have weight=1.0 in SE(3).")

        .def("isLocked", &se3ss_is_locked,
            "Return True — SE3StateSpace is always locked.\n\n"
            "C++: space->isLocked()")

        .def("getName", &SE3StateSpace::getName,
            "Return the name of this space.\n"
            "C++: space->getName()")

        .def("setName", &SE3StateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n"
            "C++: space->setName(name);")

        .def("__repr__", &se3ss_repr);
}