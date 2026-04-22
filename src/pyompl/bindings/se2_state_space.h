#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience alias
using SE2ST = SE2StateSpace::StateType;

// =============================================================================
// WHAT IS SE(2)?
// --------------
// SE(2) = Special Euclidean Group in 2 dimensions.
// It represents ALL possible positions AND orientations of a rigid body
// moving in a 2D plane.
//
// A state in SE(2) has THREE components:
//   x   — horizontal position (from RealVectorStateSpace)
//   y   — vertical position   (from RealVectorStateSpace)
//   yaw — heading angle       (from SO2StateSpace, range (-π, π])
//
// INTERNALLY SE(2) is a COMPOUND STATE SPACE:
//   subspace 0: RealVectorStateSpace(2) with weight 1.0  → (x, y)
//   subspace 1: SO2StateSpace()          with weight 0.5  → yaw
//
// The compound distance formula is:
//   d = w0 * d_position + w1 * d_yaw
//     = 1.0 * euclidean(p1, p2) + 0.5 * arc_distance(yaw1, yaw2)
//
// WHY COMPOUND?
//   Position (R^2) and orientation (SO(2)) have different topologies.
//   By composing them, OMPL handles each correctly:
//     - Position: Euclidean distance, linear interpolation
//     - Orientation: circular distance, arc interpolation (angle wrapping)
//
// USE FOR: wheeled robot (car, diff-drive), mobile manipulator base,
//          any agent that translates and rotates in 2D.
//
// C++ header: ompl/base/spaces/SE2StateSpace.h
// C++ class:  ompl::base::SE2StateSpace
// Inherits:   CompoundStateSpace → StateSpace
// =============================================================================

// -----------------------------------------------------------------------------
// SE2StateSpace::StateType helpers
// -----------------------------------------------------------------------------

static double se2st_get_x(const SE2ST &s)   { return s.getX(); }
static double se2st_get_y(const SE2ST &s)   { return s.getY(); }
static double se2st_get_yaw(const SE2ST &s) { return s.getYaw(); }

static void se2st_set_x(SE2ST &s, double v)   { s.setX(v); }
static void se2st_set_y(SE2ST &s, double v)   { s.setY(v); }
static void se2st_set_yaw(SE2ST &s, double v) { s.setYaw(v); }

static void se2st_set_xy(SE2ST &s, double x, double y)
{
    // C++: state->setXY(x, y);
    // Sets both x and y in one call. Convenience wrapper.
    s.setXY(x, y);
}

static std::string se2st_repr(const SE2ST &s)
{
    return "<SE2State x=" + std::to_string(s.getX()) +
           " y=" + std::to_string(s.getY()) +
           " yaw=" + std::to_string(s.getYaw()) + ">";
}

// -----------------------------------------------------------------------------
// SE2StateSpace helpers
// -----------------------------------------------------------------------------

static void se2ss_set_bounds(SE2StateSpace &s, const RealVectorBounds &b)
{
    // C++: space->setBounds(bounds);
    // Sets the position (x, y) bounds.
    // ONLY applies to the position subspace (RealVectorStateSpace).
    // The yaw component has no bounds — SO(2) handles wrapping automatically.
    s.setBounds(b);
}

static const RealVectorBounds &se2ss_get_bounds(const SE2StateSpace &s)
{
    // C++: space->getBounds()
    // Returns the bounds of the position subspace.
    return s.getBounds();
}

static SE2ST *se2ss_alloc_state(SE2StateSpace &s)
{
    // C++: space->allocState()->as<SE2StateSpace::StateType>()
    // Allocates a new SE(2) state. Memory managed by the space.
    // The state is uninitialised — set x, y, yaw before using.
    // Always call freeState() when done.
    return s.allocState()->as<SE2ST>();
}

static void se2ss_free_state(SE2StateSpace &s, SE2ST *st)
{
    // C++: space->freeState(state);
    // Frees memory allocated by allocState().
    s.freeState(static_cast<State *>(st));
}

static SE2ST *se2ss_clone_state(const SE2StateSpace &s, const SE2ST *src)
{
    // C++: space->cloneState(src)->as<SE2StateSpace::StateType>()
    // Allocates a new state and copies src. Must be freed with freeState().
    return s.cloneState(static_cast<const State *>(src))->as<SE2ST>();
}

static void se2ss_copy_state(const SE2StateSpace &s, SE2ST *dst, const SE2ST *src)
{
    // C++: space->copyState(destination, source);
    // Copies all 3 components (x, y, yaw) from src to dst.
    s.copyState(static_cast<State *>(dst),
                static_cast<const State *>(src));
}

static double se2ss_distance(const SE2StateSpace &s, const SE2ST *s1, const SE2ST *s2)
{
    // C++: space->distance(state1, state2)
    // Weighted compound distance:
    //   d = 1.0 * euclidean((x1,y1), (x2,y2))  +  0.5 * arc_distance(yaw1, yaw2)
    // This accounts for BOTH how far the robot moved AND how much it rotated.
    return s.distance(static_cast<const State *>(s1),
                      static_cast<const State *>(s2));
}

static bool se2ss_equal_states(const SE2StateSpace &s, const SE2ST *s1, const SE2ST *s2)
{
    // C++: space->equalStates(state1, state2)
    // Returns True if x, y, AND yaw are all exactly equal.
    return s.equalStates(static_cast<const State *>(s1),
                         static_cast<const State *>(s2));
}

static void se2ss_interpolate(const SE2StateSpace &s,
                               const SE2ST *from, const SE2ST *to,
                               double t, SE2ST *result)
{
    // C++: space->interpolate(from, to, t, result);
    // Interpolates position linearly AND yaw along shortest arc.
    //   result.x   = from.x   + t*(to.x   - from.x)
    //   result.y   = from.y   + t*(to.y   - from.y)
    //   result.yaw = SO2 arc interpolation at t
    // t=0 → from,  t=1 → to,  t=0.5 → midpoint.
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}

static bool se2ss_satisfies_bounds(const SE2StateSpace &s, const SE2ST *st)
{
    // C++: space->satisfiesBounds(state)
    // Returns True if x and y are within the set position bounds
    // AND yaw is in (-π, π].
    return s.satisfiesBounds(static_cast<const State *>(st));
}

static void se2ss_enforce_bounds(const SE2StateSpace &s, SE2ST *st)
{
    // C++: space->enforceBounds(state);
    // Clamps x, y to position bounds, AND wraps yaw to (-π, π].
    s.enforceBounds(static_cast<State *>(st));
}

static double se2ss_get_maximum_extent(const SE2StateSpace &s)
{
    // C++: space->getMaximumExtent()
    // Returns the weighted sum of max extents of both subspaces:
    //   1.0 * max_position_distance + 0.5 * π
    return s.getMaximumExtent();
}

static double se2ss_get_measure(const SE2StateSpace &s)
{
    // C++: space->getMeasure()
    // Returns the weighted volume of SE(2):
    //   area_of_position_bounds * 2π  (the full rotation range)
    return s.getMeasure();
}

static unsigned int se2ss_get_dimension(const SE2StateSpace &s)
{
    // C++: space->getDimension()
    // Returns 3: 2 for position (x, y) + 1 for orientation (yaw).
    return s.getDimension();
}

static StateSamplerPtr se2ss_alloc_default_sampler(const SE2StateSpace &s)
{
    // C++: space->allocDefaultStateSampler()
    // Returns a CompoundStateSampler that independently samples:
    //   - (x, y) uniformly from the position bounds
    //   - yaw uniformly from (-π, π]
    return s.allocDefaultStateSampler();
}

static StateSamplerPtr se2ss_alloc_state_sampler(const SE2StateSpace &s)
{
    // C++: space->allocStateSampler()
    return s.allocStateSampler();
}

static std::string se2ss_print_state(const SE2StateSpace &s, const SE2ST *st)
{
    // C++: space->printState(state, std::cout);
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}

static std::string se2ss_print_settings(const SE2StateSpace &s)
{
    // C++: space->printSettings(std::cout);
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}

static void se2ss_register_projections(SE2StateSpace &s)
{
    // C++: space->registerProjections();
    // Registers the default SE(2) projection: maps to R^2 using (x, y).
    // Called automatically by setup().
    s.registerProjections();
}

static void se2ss_setup(SE2StateSpace &s)
{
    // C++: space->setup();
    // Finalises the space. Always call after setBounds().
    s.setup();
}

static unsigned int se2ss_get_serialization_length(const SE2StateSpace &s)
{
    // C++: space->getSerializationLength()
    // Returns 3 * sizeof(double) = 24 bytes (x, y, yaw).
    return s.getSerializationLength();
}

static std::vector<double> se2ss_copy_to_reals(const SE2StateSpace &s, const SE2ST *st)
{
    // C++: space->copyToReals(reals, source);
    // Returns [x, y, yaw] as a 3-element Python list.
    const_cast<SE2StateSpace &>(s).computeLocations();
    std::vector<double> reals;
    s.copyToReals(reals, static_cast<const State *>(st));
    return reals;
}

static void se2ss_copy_from_reals(const SE2StateSpace &s,
                                   SE2ST *dst,
                                   const std::vector<double> &reals)
{
    // C++: space->copyFromReals(destination, reals);
    // Sets state from a 3-element list [x, y, yaw].
    const_cast<SE2StateSpace &>(s).computeLocations();
    s.copyFromReals(static_cast<State *>(dst), reals);
}

// CompoundStateSpace subspace access helpers
static unsigned int se2ss_get_subspace_count(const SE2StateSpace &s)
{
    // C++: space->getSubspaceCount()
    // Returns 2: subspace 0 = RealVectorStateSpace(2), subspace 1 = SO2StateSpace.
    return s.getSubspaceCount();
}

static const StateSpacePtr &se2ss_get_subspace(const SE2StateSpace &s, unsigned int i)
{
    // C++: space->getSubspace(i)
    // Returns subspace i. i=0 → position space, i=1 → yaw space.
    return s.getSubspace(i);
}

static double se2ss_get_subspace_weight(const SE2StateSpace &s, unsigned int i)
{
    // C++: space->getSubspaceWeight(i)
    // Returns weight of subspace i. Position weight=1.0, yaw weight=0.5.
    // Used in compound distance formula.
    return s.getSubspaceWeight(i);
}

static bool se2ss_is_locked(const SE2StateSpace &s)
{
    // C++: space->isLocked()
    // Returns True for SE2StateSpace (locked in constructor — no more subspaces).
    return s.isLocked();
}

static std::string se2ss_repr(const SE2StateSpace &s)
{
    return "<SE2StateSpace name=" + s.getName() + ">";
}

static void se2ss_sample_uniform(SE2StateSpace &s, SE2ST *st)
{
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniform(static_cast<State *>(st));
}

static void se2ss_sample_uniform_near(SE2StateSpace &s, SE2ST *st,
                                       const SE2ST *near, double dist)
{
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleUniformNear(static_cast<State *>(st),
                               static_cast<const State *>(near), dist);
}

static void se2ss_sample_gaussian(SE2StateSpace &s, SE2ST *st,
                                   const SE2ST *mean, double stddev)
{
    auto sampler = s.allocDefaultStateSampler();
    sampler->sampleGaussian(static_cast<State *>(st),
                            static_cast<const State *>(mean), stddev);
}

// =============================================================================
// Binding function
// =============================================================================

inline void bind_se2_state_space(py::module_ &m)
{
    // -------------------------------------------------------------------------
    // SE2StateSpace::StateType
    // -------------------------------------------------------------------------
    py::class_<SE2ST>(m, "SE2State",
        "A state in SE(2): position (x, y) + orientation (yaw).\n\n"
        "C++ type:   ompl::base::SE2StateSpace::StateType\n"
        "C++ header: ompl/base/spaces/SE2StateSpace.h\n"
        "Inherits:   CompoundStateSpace::StateType\n\n"
        "WHAT IS AN SE(2) STATE?\n"
        "It represents the complete pose of a rigid body in 2D:\n"
        "  x   — horizontal position (metres, or any unit)\n"
        "  y   — vertical position\n"
        "  yaw — heading angle in radians, range (-π, π]\n\n"
        "INTERNAL STRUCTURE:\n"
        "  The state is compound:\n"
        "  substate[0] = RealVectorStateSpace::StateType → stores (x, y)\n"
        "  substate[1] = SO2StateSpace::StateType        → stores yaw\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocState() to get one.")

        .def_property("x", &se2st_get_x, &se2st_set_x,
            "Horizontal position component.\n"
            "C++: state->getX() / state->setX(v)\n"
            "Internally: state->as<RealVectorStateSpace::StateType>(0)->values[0]")

        .def_property("y", &se2st_get_y, &se2st_set_y,
            "Vertical position component.\n"
            "C++: state->getY() / state->setY(v)\n"
            "Internally: state->as<RealVectorStateSpace::StateType>(0)->values[1]")

        .def_property("yaw", &se2st_get_yaw, &se2st_set_yaw,
            "Heading angle in radians, range (-π, π].\n"
            "C++: state->getYaw() / state->setYaw(v)\n"
            "Internally: state->as<SO2StateSpace::StateType>(1)->value\n\n"
            "0.0   = facing East (positive x direction)\n"
            "π/2   = facing North (positive y direction)\n"
            "π/-π  = facing West\n"
            "-π/2  = facing South")

        .def("setXY", &se2st_set_xy, py::arg("x"), py::arg("y"),
            "Set both x and y in one call.\n\n"
            "C++: state->setXY(x, y);\n\n"
            "Convenience wrapper for setX(x); setY(y);")

        .def("getX",   &se2st_get_x,
            "Get the x (horizontal) position.\n"
            "C++: state->getX()")

        .def("getY",   &se2st_get_y,
            "Get the y (vertical) position.\n"
            "C++: state->getY()")

        .def("getYaw", &se2st_get_yaw,
            "Get the yaw (heading) angle in radians.\n"
            "C++: state->getYaw()")

        .def("setX",   &se2st_set_x,   py::arg("x"),
            "Set the x (horizontal) position.\n"
            "C++: state->setX(x);")

        .def("setY",   &se2st_set_y,   py::arg("y"),
            "Set the y (vertical) position.\n"
            "C++: state->setY(y);")

        .def("setYaw", &se2st_set_yaw, py::arg("yaw"),
            "Set the yaw (heading) angle in radians.\n"
            "C++: state->setYaw(yaw);\n\n"
            "Values outside (-π, π] are accepted but you should call\n"
            "space.enforceBounds(state) to normalise.")

        .def("__repr__", &se2st_repr);

    // -------------------------------------------------------------------------
    // SE2StateSpace
    // -------------------------------------------------------------------------
    py::class_<SE2StateSpace, CompoundStateSpace,
               std::shared_ptr<SE2StateSpace>>(m, "SE2StateSpace",
        "State space representing SE(2) — 2D position + heading.\n\n"
        "C++ header: ompl/base/spaces/SE2StateSpace.h\n"
        "C++ class:  ompl::base::SE2StateSpace\n"
        "Inherits:   CompoundStateSpace → StateSpace\n\n"
        "SE(2) = Special Euclidean Group in 2D.\n"
        "Represents all possible poses (position + orientation) in a plane.\n\n"
        "INTERNAL STRUCTURE (CompoundStateSpace):\n"
        "  Subspace 0: RealVectorStateSpace(2), weight=1.0  → (x, y)\n"
        "  Subspace 1: SO2StateSpace(),          weight=0.5  → yaw\n\n"
        "DISTANCE FORMULA:\n"
        "  d = 1.0 * euclidean(p1, p2) + 0.5 * arc_distance(yaw1, yaw2)\n\n"
        "INTERPOLATION:\n"
        "  Position: linear  |  Yaw: shortest arc (angle-aware)\n\n"
        "USE FOR: wheeled robots, mobile bases, any 2D rigid body.\n\n"
        "SETUP REQUIRED:\n"
        "  1. space = SE2StateSpace()\n"
        "  2. space.setBounds(bounds)   ← set position limits\n"
        "  3. space.setup()\n"
        "  Yaw bounds are automatic (SO(2) handles wrapping).")

        .def(py::init<>(),
            "Construct an SE(2) state space.\n\n"
            "C++: auto space = std::make_shared<SE2StateSpace>();\n\n"
            "Automatically creates:\n"
            "  - RealVectorStateSpace(2) with weight 1.0\n"
            "  - SO2StateSpace() with weight 0.5\n"
            "The space is locked (no further subspaces can be added).\n"
            "You MUST call setBounds() to set position limits before use.")

        .def("setBounds", &se2ss_set_bounds, py::arg("bounds"),
            "Set the position (x, y) bounds.\n\n"
            "C++: space->setBounds(bounds);\n\n"
            "Sets bounds for the RealVectorStateSpace(2) subspace.\n"
            "MUST be called before using the space.\n"
            "Yaw bounds are NOT set here — SO(2) is automatically bounded.\n\n"
            "Example:\n"
            "  bounds = RealVectorBounds(2)\n"
            "  bounds.setLow(-10.0)    # x and y >= -10\n"
            "  bounds.setHigh(10.0)    # x and y <= 10\n"
            "  space.setBounds(bounds)")

        .def("getBounds", &se2ss_get_bounds,
            py::return_value_policy::reference_internal,
            "Return the current position bounds.\n\n"
            "C++: space->getBounds()\n\n"
            "Returns the RealVectorBounds of the (x, y) subspace.\n"
            "Yaw has no bounds object — it is always (-π, π].")

        .def("getDimension", &se2ss_get_dimension,
            "Return the total dimension: 3 (x=1, y=1, yaw=1).\n\n"
            "C++: space->getDimension()\n\n"
            "Even though SE(2) is a compound space with 2 subspaces,\n"
            "its total dimension is 3: 2 position + 1 rotation.")

        .def("getMaximumExtent", &se2ss_get_maximum_extent,
            "Return the weighted maximum extent of SE(2).\n\n"
            "C++: space->getMaximumExtent()\n\n"
            "= 1.0 * max_position_distance + 0.5 * π\n"
            "Where max_position_distance is the diagonal of the position bounds.")

        .def("getMeasure", &se2ss_get_measure,
            "Return the weighted volume of SE(2).\n\n"
            "C++: space->getMeasure()\n\n"
            "= area_of_position_bounds * 2π")

        .def("satisfiesBounds", &se2ss_satisfies_bounds, py::arg("state"),
            "Return True if x, y are within bounds AND yaw is in (-π, π].\n\n"
            "C++: space->satisfiesBounds(state)")

        .def("enforceBounds", &se2ss_enforce_bounds, py::arg("state"),
            "Clamp x, y to bounds and wrap yaw to (-π, π].\n\n"
            "C++: space->enforceBounds(state);\n\n"
            "Combines position clamping (R^2) and angle wrapping (SO(2)).")

        .def("copyState", &se2ss_copy_state,
            py::arg("destination"), py::arg("source"),
            "Copy all 3 components (x, y, yaw) from source to destination.\n\n"
            "C++: space->copyState(destination, source);")

        .def("cloneState", &se2ss_clone_state, py::arg("source"),
            py::return_value_policy::reference_internal,
            "Allocate a new state and copy source into it.\n\n"
            "C++: space->cloneState(src)->as<SE2StateSpace::StateType>()\n\n"
            "Must be freed with freeState().")

        .def("copyToReals", &se2ss_copy_to_reals, py::arg("source"),
            "Return [x, y, yaw] as a 3-element Python list.\n\n"
            "C++: space->copyToReals(reals, source);\n\n"
            "Requires setup() to have been called first.")

        .def("copyFromReals", &se2ss_copy_from_reals,
            py::arg("destination"), py::arg("reals"),
            "Set state from a 3-element list [x, y, yaw].\n\n"
            "C++: space->copyFromReals(destination, reals);\n\n"
            "Requires setup() to have been called first.")

        .def("getSerializationLength", &se2ss_get_serialization_length,
            "Return bytes to serialise one SE(2) state.\n\n"
            "C++: space->getSerializationLength()\n\n"
            "Returns 24 = 3 * sizeof(double).")

        .def("distance", &se2ss_distance, py::arg("state1"), py::arg("state2"),
            "Return the weighted compound distance between two SE(2) states.\n\n"
            "C++: space->distance(state1, state2)\n\n"
            "Formula:\n"
            "  d = 1.0 * euclidean((x1,y1),(x2,y2)) + 0.5 * arc(yaw1,yaw2)\n\n"
            "This combines HOW FAR the robot moved (position) and\n"
            "HOW MUCH it rotated (yaw), weighted by subspace weights.")

        .def("equalStates", &se2ss_equal_states,
            py::arg("state1"), py::arg("state2"),
            "Return True if x, y, AND yaw are all exactly equal.\n\n"
            "C++: space->equalStates(state1, state2)")

        .def("interpolate", &se2ss_interpolate,
            py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
            "Interpolate between two SE(2) states at fraction t.\n\n"
            "C++: space->interpolate(from, to, t, result);\n\n"
            "t=0 → from,  t=1 → to,  t=0.5 → midpoint.\n\n"
            "Position interpolated LINEARLY:\n"
            "  result.x = from.x + t*(to.x - from.x)\n"
            "  result.y = from.y + t*(to.y - from.y)\n\n"
            "Yaw interpolated via SHORTEST ARC (SO(2)):\n"
            "  result.yaw = SO2 arc interpolation\n\n"
            "'result' must be pre-allocated.")

        .def("allocState", &se2ss_alloc_state,
            py::return_value_policy::reference_internal,
            "Allocate a new SE(2) state. Memory managed by the space.\n\n"
            "C++: space->allocState()->as<SE2StateSpace::StateType>()\n\n"
            "State is uninitialised. Set x, y, yaw before using.\n"
            "Always call freeState() when done.")

        .def("freeState", &se2ss_free_state, py::arg("state"),
            "Free a state previously allocated by allocState().\n\n"
            "C++: space->freeState(state);")

        .def("allocDefaultStateSampler", &se2ss_alloc_default_sampler,
            "Allocate the default SE(2) sampler.\n\n"
            "C++: space->allocDefaultStateSampler()\n\n"
            "Returns a CompoundStateSampler that samples:\n"
            "  - (x, y) uniformly from position bounds\n"
            "  - yaw uniformly from (-π, π]")

        .def("allocStateSampler", &se2ss_alloc_state_sampler,
            "Allocate a sampler (custom if set, else default).\n\n"
            "C++: space->allocStateSampler()")


        .def("sampleUniform", &se2ss_sample_uniform, py::arg("state"),
            "Sample a random SE(2) state within the bounds.\n"
            "Samples (x,y) uniformly from position bounds and yaw from (-π,π].\n"
            "C++: space->allocDefaultStateSampler()->sampleUniform(state);")

        .def("sampleUniformNear", &se2ss_sample_uniform_near,
            py::arg("state"), py::arg("near"), py::arg("distance"),
            "Sample a random SE(2) state near 'near' within 'distance'.\n"
            "C++: space->allocDefaultStateSampler()->sampleUniformNear(state,near,dist);")

        .def("sampleGaussian", &se2ss_sample_gaussian,
            py::arg("state"), py::arg("mean"), py::arg("stdDev"),
            "Sample a random SE(2) state from Gaussian centred at 'mean'.\n"
            "C++: space->allocDefaultStateSampler()->sampleGaussian(state,mean,std);")


        

        .def("printState", &se2ss_print_state, py::arg("state"),
            "Return a string showing x, y, and yaw.\n\n"
            "C++: space->printState(state, std::cout);")

        .def("printSettings", &se2ss_print_settings,
            "Return a string describing the space settings.\n\n"
            "C++: space->printSettings(std::cout);")

        .def("registerProjections", &se2ss_register_projections,
            "Register the default SE(2) projection.\n\n"
            "C++: space->registerProjections();\n\n"
            "Maps SE(2) states to R^2 using the (x, y) position.\n"
            "Called automatically by setup().")

        .def("setup", &se2ss_setup,
            "Finalise the space. Call after setBounds().\n\n"
            "C++: space->setup();\n\n"
            "Called automatically by SpaceInformation.")

        // CompoundStateSpace methods exposed for SE2
        .def("getSubspaceCount", &se2ss_get_subspace_count,
            "Return the number of subspaces: always 2 for SE(2).\n\n"
            "C++: space->getSubspaceCount()\n\n"
            "Subspace 0: RealVectorStateSpace(2) — position\n"
            "Subspace 1: SO2StateSpace()          — yaw")

        .def("getSubspace", &se2ss_get_subspace, py::arg("index"),
            "Return the subspace at index (0=position, 1=yaw).\n\n"
            "C++: space->getSubspace(index)")

        .def("getSubspaceWeight", &se2ss_get_subspace_weight, py::arg("index"),
            "Return the weight of subspace i.\n\n"
            "C++: space->getSubspaceWeight(index)\n\n"
            "Position (index=0): weight=1.0\n"
            "Yaw     (index=1): weight=0.5\n"
            "These weights are used in the compound distance formula.")

        .def("isLocked", &se2ss_is_locked,
            "Return True — SE2StateSpace is always locked.\n\n"
            "C++: space->isLocked()\n\n"
            "The SE(2) constructor calls lock() to prevent adding\n"
            "further subspaces. You cannot modify the structure.")

        .def("getName", &SE2StateSpace::getName,
            "Return the name of this space.\n"
            "C++: space->getName()")

        .def("setName", &SE2StateSpace::setName, py::arg("name"),
            "Set a custom name for this space.\n"
            "C++: space->setName(name);")

        .def("__repr__", &se2ss_repr);
}