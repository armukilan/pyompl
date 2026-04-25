#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSampler.h>
// #include <ompl/base/CompoundState.h>   // add this line

namespace py = pybind11;
using namespace ompl::base;

// ---------------------------------------------------------------------------
// Helper free functions for StateSpace
// (MSVC requires free functions instead of inline lambdas in .def() chains)
// ---------------------------------------------------------------------------

// --- StateSpace helpers ---

static bool ss_is_compound(const StateSpace &s)          { return s.isCompound(); }
static bool ss_is_discrete(const StateSpace &s)          { return s.isDiscrete(); }
static bool ss_is_hybrid(const StateSpace &s)            { return s.isHybrid(); }
static bool ss_is_metric_space(const StateSpace &s)      { return s.isMetricSpace(); }
static bool ss_has_sym_distance(const StateSpace &s)     { return s.hasSymmetricDistance(); }
static bool ss_has_sym_interpolate(const StateSpace &s)  { return s.hasSymmetricInterpolate(); }
static const std::string &ss_get_name(const StateSpace &s) { return s.getName(); }
static void ss_set_name(StateSpace &s, const std::string &n) { s.setName(n); }
static int  ss_get_type(const StateSpace &s)             { return s.getType(); }

static bool ss_includes_ptr(const StateSpace &s, const StateSpacePtr &other)
{
    return s.includes(other);
}
static bool ss_covers_ptr(const StateSpace &s, const StateSpacePtr &other)
{
    return s.covers(other);
}

static double ss_get_longest_valid_segment_fraction(const StateSpace &s)
{
    return s.getLongestValidSegmentFraction();
}
static void ss_set_longest_valid_segment_fraction(StateSpace &s, double f)
{
    s.setLongestValidSegmentFraction(f);
}
static unsigned int ss_valid_segment_count(const StateSpace &s,
                                           const State *s1, const State *s2)
{
    return s.validSegmentCount(s1, s2);
}
static void ss_set_valid_segment_count_factor(StateSpace &s, unsigned int f)
{
    s.setValidSegmentCountFactor(f);
}
static unsigned int ss_get_valid_segment_count_factor(const StateSpace &s)
{
    return s.getValidSegmentCountFactor();
}
static double ss_get_longest_valid_segment_length(const StateSpace &s)
{
    return s.getLongestValidSegmentLength();
}
static std::vector<int> ss_compute_signature(const StateSpace &s)
{
    std::vector<int> sig;
    s.computeSignature(sig);
    return sig;
}
static unsigned int ss_get_dimension(const StateSpace &s)      { return s.getDimension(); }
static double ss_get_maximum_extent(const StateSpace &s)       { return s.getMaximumExtent(); }
static double ss_get_measure(const StateSpace &s)              { return s.getMeasure(); }
static void   ss_enforce_bounds(StateSpace &s, State *st)      { s.enforceBounds(st); }
static bool   ss_satisfies_bounds(const StateSpace &s, const State *st) { return s.satisfiesBounds(st); }
static void   ss_copy_state(const StateSpace &s, State *dst, const State *src) { s.copyState(dst, src); }
static State *ss_clone_state(const StateSpace &s, const State *src) { return s.cloneState(src); }
static double ss_distance(const StateSpace &s, const State *s1, const State *s2) { return s.distance(s1, s2); }
static bool   ss_equal_states(const StateSpace &s, const State *s1, const State *s2) { return s.equalStates(s1, s2); }
static void   ss_interpolate(const StateSpace &s, const State *from, const State *to, double t, State *st)
{
    s.interpolate(from, to, t, st);
}
static StateSamplerPtr ss_alloc_default_sampler(const StateSpace &s) { return s.allocDefaultStateSampler(); }
static StateSamplerPtr ss_alloc_state_sampler(const StateSpace &s)   { return s.allocStateSampler(); }
static State *ss_alloc_state(const StateSpace &s)                    { return s.allocState(); }
static void   ss_free_state(const StateSpace &s, State *st)          { s.freeState(st); }

static std::vector<double> ss_copy_to_reals(const StateSpace &s, const State *src)
{
    std::vector<double> reals;
    s.copyToReals(reals, src);
    return reals;
}
static void ss_copy_from_reals(const StateSpace &s, State *dst, const std::vector<double> &reals)
{
    s.copyFromReals(dst, reals);
}

static ProjectionEvaluatorPtr ss_get_projection(const StateSpace &s, const std::string &name)
{
    return s.getProjection(name);
}
static ProjectionEvaluatorPtr ss_get_default_projection(const StateSpace &s)
{
    return s.getDefaultProjection();
}
static bool ss_has_projection(const StateSpace &s, const std::string &name) { return s.hasProjection(name); }
static bool ss_has_default_projection(const StateSpace &s)                  { return s.hasDefaultProjection(); }
static void ss_register_projections(StateSpace &s)                          { s.registerProjections(); }

static std::string ss_print_state_str(const StateSpace &s, const State *st)
{
    std::ostringstream oss;
    s.printState(st, oss);
    return oss.str();
}
static std::string ss_print_settings_str(const StateSpace &s)
{
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}
static std::string ss_print_projections_str(const StateSpace &s)
{
    std::ostringstream oss;
    s.printProjections(oss);
    return oss.str();
}
static void ss_sanity_checks(const StateSpace &s) { s.sanityChecks(); }
static void ss_setup(StateSpace &s)               { s.setup(); }
static void ss_compute_locations(StateSpace &s)   { s.computeLocations(); }

static std::string ss_diagram_str(const StateSpace &s)
{
    std::ostringstream oss;
    s.diagram(oss);
    return oss.str();
}
static std::string ss_list_str(const StateSpace &s)
{
    std::ostringstream oss;
    s.list(oss);
    return oss.str();
}

static std::vector<std::string> ss_get_common_subspaces(const StateSpace &s, const StateSpacePtr &other)
{
    std::vector<std::string> result;
    s.getCommonSubspaces(other, result);
    return result;
}

// --- CompoundStateSpace helpers ---

static void css_add_subspace(CompoundStateSpace &s, const StateSpacePtr &comp, double weight)
{
    s.addSubspace(comp, weight);
}
static unsigned int css_get_subspace_count(const CompoundStateSpace &s)
{
    return s.getSubspaceCount();
}
static const StateSpacePtr &css_get_subspace_by_index(const CompoundStateSpace &s, unsigned int i)
{
    return s.getSubspace(i);
}
static const StateSpacePtr &css_get_subspace_by_name(const CompoundStateSpace &s, const std::string &name)
{
    return s.getSubspace(name);
}
static unsigned int css_get_subspace_index(const CompoundStateSpace &s, const std::string &name)
{
    return s.getSubspaceIndex(name);
}
static bool css_has_subspace(const CompoundStateSpace &s, const std::string &name)
{
    return s.hasSubspace(name);
}
static double css_get_subspace_weight_by_index(const CompoundStateSpace &s, unsigned int i)
{
    return s.getSubspaceWeight(i);
}
static double css_get_subspace_weight_by_name(const CompoundStateSpace &s, const std::string &name)
{
    return s.getSubspaceWeight(name);
}
static void css_set_subspace_weight_by_index(CompoundStateSpace &s, unsigned int i, double w)
{
    s.setSubspaceWeight(i, w);
}
static void css_set_subspace_weight_by_name(CompoundStateSpace &s, const std::string &name, double w)
{
    s.setSubspaceWeight(name, w);
}
static const std::vector<StateSpacePtr> &css_get_subspaces(const CompoundStateSpace &s)
{
    return s.getSubspaces();
}
static const std::vector<double> &css_get_subspace_weights(const CompoundStateSpace &s)
{
    return s.getSubspaceWeights();
}
static bool css_is_locked(const CompoundStateSpace &s) { return s.isLocked(); }
static void css_lock(CompoundStateSpace &s)            { s.lock(); }

// ---------------------------------------------------------------------------
// Binding function
// ---------------------------------------------------------------------------

inline void bind_state_space(py::module_ &m)
{
    // ------------------------------------------------------------------ //
    //  SanityChecks flags enum
    // ------------------------------------------------------------------ //
    py::enum_<StateSpace::SanityChecks>(m, "SanityChecks",
        "Bit-mask flags for StateSpace::sanityChecks().\n"
        "C++: ompl::base::StateSpace::SanityChecks")
        .value("STATESPACE_DISTANCE_DIFFERENT_STATES",
               StateSpace::STATESPACE_DISTANCE_DIFFERENT_STATES,
               "Check distances between non-equal states are strictly positive.")
        .value("STATESPACE_DISTANCE_SYMMETRIC",
               StateSpace::STATESPACE_DISTANCE_SYMMETRIC,
               "Check distance function is symmetric.")
        .value("STATESPACE_INTERPOLATION",
               StateSpace::STATESPACE_INTERPOLATION,
               "Check interpolate() works as expected.")
        .value("STATESPACE_TRIANGLE_INEQUALITY",
               StateSpace::STATESPACE_TRIANGLE_INEQUALITY,
               "Check triangle inequality holds.")
        .value("STATESPACE_DISTANCE_BOUND",
               StateSpace::STATESPACE_DISTANCE_BOUND,
               "Check distance is bounded by getMaximumExtent().")
        .value("STATESPACE_RESPECT_BOUNDS",
               StateSpace::STATESPACE_RESPECT_BOUNDS,
               "Check sampled states are always within bounds.")
        .value("STATESPACE_ENFORCE_BOUNDS_NO_OP",
               StateSpace::STATESPACE_ENFORCE_BOUNDS_NO_OP,
               "Check enforceBounds() does not modify in-bounds states.")
        .value("STATESPACE_SERIALIZATION",
               StateSpace::STATESPACE_SERIALIZATION,
               "Check serialize/deserialize work correctly.")
        .export_values();

    py::class_<State, std::unique_ptr<State, py::nodelete>>(m, "State",
        "Opaque base state handle. Use through the space methods only.\n"
        "C++: ompl::base::State");

    py::class_<CompoundState, State, std::unique_ptr<CompoundState, py::nodelete>>(m, "CompoundState",
        "Opaque compound state handle.\n"
        "C++: ompl::base::CompoundState");
    // ------------------------------------------------------------------ //
    //  StateSpace (abstract base)
    // ------------------------------------------------------------------ //
    py::class_<StateSpace, std::shared_ptr<StateSpace>>(m, "StateSpace",
        "Abstract base class for all OMPL state spaces.\n\n"
        "C++ header: ompl/base/StateSpace.h\n"
        "C++ class:  ompl::base::StateSpace\n\n"
        "A StateSpace defines the topology of the configuration space:\n"
        "how states are represented, how distance is measured, how to\n"
        "interpolate between states, and how to sample random states.\n"
        "You never instantiate this directly — use a subclass like\n"
        "RealVectorStateSpace, SE2StateSpace, SE3StateSpace, etc.")

        // --- Type checks ---
        .def("isCompound",            &ss_is_compound,
             "Return True if this is a compound state space (made of subspaces).\n"
             "C++: space->isCompound()")
        .def("isDiscrete",            &ss_is_discrete,
             "Return True if this represents a mathematically discrete space.\n"
             "C++: space->isDiscrete()")
        .def("isHybrid",              &ss_is_hybrid,
             "Return True if this space has both discrete and continuous components.\n"
             "C++: space->isHybrid()")
        .def("isMetricSpace",         &ss_is_metric_space,
             "Return True if the distance function is a proper metric.\n"
             "C++: space->isMetricSpace()")
        .def("hasSymmetricDistance",  &ss_has_sym_distance,
             "Return True if distance(s1,s2) == distance(s2,s1).\n"
             "C++: space->hasSymmetricDistance()")
        .def("hasSymmetricInterpolate", &ss_has_sym_interpolate,
             "Return True if interpolate(from,to,t) == interpolate(to,from,1-t).\n"
             "C++: space->hasSymmetricInterpolate()")

        // --- Name and type ---
        .def("getName",               &ss_get_name,
             "Return the name of this state space.\n"
             "C++: space->getName()")
        .def("setName",               &ss_set_name,  py::arg("name"),
             "Set the name of this state space.\n"
             "C++: space->setName(name);")
        .def("getType",               &ss_get_type,
             "Return the integer type identifier for this space.\n"
             "Corresponds to StateSpaceType enum values in OMPL.\n"
             "C++: space->getType()")

        // --- Inclusion and coverage ---
        .def("includes",              &ss_includes_ptr, py::arg("other"),
             "Return True if 'other' is a subspace of (or equal to) this space.\n"
             "C++: space->includes(other)")
        .def("covers",                &ss_covers_ptr,   py::arg("other"),
             "Return True if all subspaces of 'other' are included in this space.\n"
             "C++: space->covers(other)")

        // --- Valid segment / resolution ---
        .def("getLongestValidSegmentFraction",  &ss_get_longest_valid_segment_fraction,
             "Return the fraction of max extent used as the longest valid segment.\n"
             "C++: space->getLongestValidSegmentFraction()")
        .def("setLongestValidSegmentFraction",  &ss_set_longest_valid_segment_fraction,
             py::arg("segmentFraction"),
             "Set the longest valid segment as a fraction of max extent.\n"
             "Takes effect after setup() is called.\n"
             "C++: space->setLongestValidSegmentFraction(fraction);")
        .def("validSegmentCount",               &ss_valid_segment_count,
             py::arg("state1"), py::arg("state2"),
             "Count how many valid-length segments fit on the path from state1 to state2.\n"
             "C++: space->validSegmentCount(state1, state2)")
        .def("setValidSegmentCountFactor",      &ss_set_valid_segment_count_factor,
             py::arg("factor"),
             "Multiply validSegmentCount() results by this factor. Default is 1.\n"
             "Higher values = finer resolution validation. Takes effect immediately.\n"
             "C++: space->setValidSegmentCountFactor(factor);")
        .def("getValidSegmentCountFactor",      &ss_get_valid_segment_count_factor,
             "Return the current validSegmentCount multiplier.\n"
             "C++: space->getValidSegmentCountFactor()")
        .def("getLongestValidSegmentLength",    &ss_get_longest_valid_segment_length,
             "Return the longest valid segment length computed at setup() time.\n"
             "C++: space->getLongestValidSegmentLength()")

        // --- Signature ---
        .def("computeSignature",      &ss_compute_signature,
             "Return an int array uniquely identifying the structure of this space.\n"
             "C++: space->computeSignature(signature);")

        // --- Core space operations ---
        .def("getDimension",          &ss_get_dimension,
             "Return the number of dimensions in this space.\n"
             "C++: space->getDimension()")
        .def("getMaximumExtent",      &ss_get_maximum_extent,
             "Return the maximum possible distance between any two states.\n"
             "C++: space->getMaximumExtent()")
        .def("getMeasure",            &ss_get_measure,
             "Return a measure of the space (generalisation of volume).\n"
             "C++: space->getMeasure()")
        .def("enforceBounds",         &ss_enforce_bounds,    py::arg("state"),
             "Clamp state values to lie within the space bounds.\n"
             "C++: space->enforceBounds(state);")
        .def("satisfiesBounds",       &ss_satisfies_bounds,  py::arg("state"),
             "Return True if the state is within the space bounds.\n"
             "C++: space->satisfiesBounds(state)")
        .def("copyState",             &ss_copy_state,
             py::arg("destination"), py::arg("source"),
             "Copy source state into destination. Memory must not overlap.\n"
             "C++: space->copyState(destination, source);")
        .def("cloneState",            &ss_clone_state,       py::arg("source"),
             py::return_value_policy::reference,
             "Allocate a new state and copy source into it. Caller must free.\n"
             "C++: space->cloneState(source)")
        .def("distance",              &ss_distance,
             py::arg("state1"), py::arg("state2"),
             "Return the distance between two states.\n"
             "C++: space->distance(state1, state2)")
        .def("equalStates",           &ss_equal_states,
             py::arg("state1"), py::arg("state2"),
             "Return True if two states are equal.\n"
             "C++: space->equalStates(state1, state2)")
        .def("interpolate",           &ss_interpolate,
             py::arg("from"), py::arg("to"), py::arg("t"), py::arg("state"),
             "Compute the state at fraction t in [0,1] between from and to.\n"
             "t=0 -> from, t=1 -> to.\n"
             "C++: space->interpolate(from, to, t, state);")

        // --- Samplers ---
        .def("allocDefaultStateSampler", &ss_alloc_default_sampler,
             "Allocate the default uniform sampler for this space.\n"
             "C++: space->allocDefaultStateSampler()")
        .def("allocStateSampler",        &ss_alloc_state_sampler,
             "Allocate a sampler (custom if set, else default).\n"
             "C++: space->allocStateSampler()")

        // --- State allocation ---
        .def("allocState",            &ss_alloc_state,
             py::return_value_policy::reference,
             "Allocate a new state. Must be freed with freeState().\n"
             "C++: space->allocState()")
        .def("freeState",             &ss_free_state,        py::arg("state"),
             "Free a previously allocated state.\n"
             "C++: space->freeState(state);")

        // --- Real value access ---
        .def("copyToReals",           &ss_copy_to_reals,     py::arg("source"),
             "Return all double values in the state as a Python list.\n"
             "C++: space->copyToReals(reals, source);")
        .def("copyFromReals",         &ss_copy_from_reals,
             py::arg("destination"), py::arg("reals"),
             "Set all double values in the state from a Python list.\n"
             "C++: space->copyFromReals(destination, reals);")

        // --- Projections ---
        .def("registerProjections",   &ss_register_projections,
             "Register the default projections for this space. Called by setup().\n"
             "C++: space->registerProjections();")
        .def("getProjection",         &ss_get_projection,    py::arg("name"),
             "Return the projection registered under the given name.\n"
             "C++: space->getProjection(name)")
        .def("getDefaultProjection",  &ss_get_default_projection,
             "Return the default projection for this space.\n"
             "C++: space->getDefaultProjection()")
        .def("hasProjection",         &ss_has_projection,    py::arg("name"),
             "Return True if a projection with the given name is registered.\n"
             "C++: space->hasProjection(name)")
        .def("hasDefaultProjection",  &ss_has_default_projection,
             "Return True if a default projection is registered.\n"
             "C++: space->hasDefaultProjection()")

        // --- Subspace operations ---
        .def("getCommonSubspaces",    &ss_get_common_subspaces, py::arg("other"),
             "Return names of subspaces that this space and 'other' have in common.\n"
             "C++: space->getCommonSubspaces(other, subspaces);")

        // --- Debug / print ---
        .def("printState",            &ss_print_state_str,   py::arg("state"),
             "Return a string representation of the given state.\n"
             "C++: space->printState(state, std::cout);")
        .def("printSettings",         &ss_print_settings_str,
             "Return a string describing this space's settings.\n"
             "C++: space->printSettings(std::cout);")
        .def("printProjections",      &ss_print_projections_str,
             "Return a string listing all registered projections.\n"
             "C++: space->printProjections(std::cout);")
        .def("diagram",               &ss_diagram_str,
             "Return a Graphviz digraph string of the containment diagram.\n"
             "C++: space->diagram(std::cout);")
        .def("list",                  &ss_list_str,
             "Return a string listing all contained state space instances.\n"
             "C++: space->list(std::cout);")
        .def("sanityChecks",          &ss_sanity_checks,
             "Run sanity checks on this space. Throws if any check fails.\n"
             "C++: space->sanityChecks();")

        // --- Setup ---
        .def("setup",                 &ss_setup,
             "Perform final setup. Called automatically by SpaceInformation.\n"
             "Registers default projections and computes segment lengths.\n"
             "C++: space->setup();")
        .def("computeLocations",      &ss_compute_locations,
             "Compute value/substate location maps. Call before getValueAddressAtName().\n"
             "C++: space->computeLocations();");

    // ------------------------------------------------------------------ //
    //  CompoundStateSpace
    // ------------------------------------------------------------------ //
    // ------------------------------------------------------------------ //
    //  StateSampler (abstract base) — must be registered before any
    //  concrete sampler like RealVectorStateSampler
    // ------------------------------------------------------------------ //
    py::class_<StateSampler, std::shared_ptr<StateSampler>>(m, "StateSampler",
        "Abstract base class for all OMPL state samplers.\n\n"
        "C++ header: ompl/base/StateSampler.h\n"
        "C++ class:  ompl::base::StateSampler\n\n"
        "Do not instantiate directly. Use space.allocStateSampler() or\n"
        "space.allocDefaultStateSampler() to get a sampler.")

        // .def("sampleUniform",
        //      [](StateSampler &s, State *st) { s.sampleUniform(st); },
        //      py::arg("state"),
        //      "Sample a state uniformly at random within the space bounds.\n"
        //      "C++: sampler->sampleUniform(state);")

        // .def("sampleUniformNear",
        //      [](StateSampler &s, State *st, const State *near, double dist) {
        //          s.sampleUniformNear(st, near, dist);
        //      },
        //      py::arg("state"), py::arg("near"), py::arg("distance"),
        //      "Sample a state near 'near' within 'distance'.\n"
        //      "C++: sampler->sampleUniformNear(state, near, distance);")

        // .def("sampleGaussian",
        //      [](StateSampler &s, State *st, const State *mean, double stddev) {
        //          s.sampleGaussian(st, mean, stddev);
        //      },
        //      py::arg("state"), py::arg("mean"), py::arg("stdDev"),
        //      "Sample a state from a Gaussian centred at 'mean'.\n"
        //      "C++: sampler->sampleGaussian(state, mean, stdDev);");
        .def("sampleUniform",
             [](StateSampler &s, py::object st) {
                 s.sampleUniform(st.cast<State *>());
             },
             py::arg("state"),
             "Sample a state uniformly at random within the space bounds.\n"
             "C++: sampler->sampleUniform(state);")

        .def("sampleUniformNear",
             [](StateSampler &s, py::object st, py::object near, double dist) {
                 s.sampleUniformNear(st.cast<State *>(),
                                     near.cast<State *>(), dist);
             },
             py::arg("state"), py::arg("near"), py::arg("distance"),
             "Sample a state near 'near' within 'distance'.\n"
             "C++: sampler->sampleUniformNear(state, near, distance);")

        .def("sampleGaussian",
             [](StateSampler &s, py::object st, py::object mean, double stddev) {
                 s.sampleGaussian(st.cast<State *>(),
                                  mean.cast<State *>(), stddev);
             },
             py::arg("state"), py::arg("mean"), py::arg("stdDev"),
             "Sample a state from a Gaussian centred at 'mean'.\n"
             "C++: sampler->sampleGaussian(state, mean, stdDev);");



        
    py::class_<CompoundStateSpace, StateSpace,
               std::shared_ptr<CompoundStateSpace>>(m, "CompoundStateSpace",
        "A state space composed of multiple subspaces.\n\n"
        "C++ header: ompl/base/StateSpace.h\n"
        "C++ class:  ompl::base::CompoundStateSpace\n\n"
        "Used internally by SE2StateSpace, SE3StateSpace, etc.\n"
        "You can also build your own compound space by adding subspaces.\n"
        "Each subspace has a weight used in distance computation.")

        .def(py::init<>(),
             "Construct an empty compound state space.\n"
             "C++: CompoundStateSpace space;")

        // --- Subspace management ---
        .def("addSubspace",           &css_add_subspace,
             py::arg("component"), py::arg("weight"),
             "Add a subspace with the given weight.\n"
             "Weight is used when computing compound distance.\n"
             "C++: space.addSubspace(component, weight);")
        .def("getSubspaceCount",      &css_get_subspace_count,
             "Return the number of subspaces.\n"
             "C++: space.getSubspaceCount()")
        // .def("getSubspace",
        //      py::overload_cast<unsigned int>(&css_get_subspace_by_index),
        .def("getSubspace",
             &css_get_subspace_by_index,
             py::arg("index"),
             "Return the subspace at the given index.\n"
             "C++: space.getSubspace(index)")
        // .def("getSubspace",
        //      py::overload_cast<const std::string &>(&css_get_subspace_by_name),
        .def("getSubspace",
             &css_get_subspace_by_name,
             py::arg("name"),
             "Return the subspace with the given name.\n"
             "C++: space.getSubspace(name)")
        .def("getSubspaceIndex",      &css_get_subspace_index,  py::arg("name"),
             "Return the index of the subspace with the given name.\n"
             "C++: space.getSubspaceIndex(name)")
        .def("hasSubspace",           &css_has_subspace,        py::arg("name"),
             "Return True if a subspace with the given name exists.\n"
             "C++: space.hasSubspace(name)")
        // .def("getSubspaceWeight",
        //      py::overload_cast<unsigned int>(&css_get_subspace_weight_by_index),
        .def("getSubspaceWeight",
             &css_get_subspace_weight_by_index,
             py::arg("index"),
             "Return the weight of the subspace at the given index.\n"
             "C++: space.getSubspaceWeight(index)")
        // .def("getSubspaceWeight",
        //      py::overload_cast<const std::string &>(&css_get_subspace_weight_by_name),
        .def("getSubspaceWeight",
             &css_get_subspace_weight_by_name,
             py::arg("name"),
             "Return the weight of the subspace with the given name.\n"
             "C++: space.getSubspaceWeight(name)")
        // .def("setSubspaceWeight",
        //      py::overload_cast<unsigned int, double>(&css_set_subspace_weight_by_index),
        .def("setSubspaceWeight",
             &css_set_subspace_weight_by_index,
             py::arg("index"), py::arg("weight"),
             "Set the weight of the subspace at the given index.\n"
             "C++: space.setSubspaceWeight(index, weight);")
        // .def("setSubspaceWeight",
        //      py::overload_cast<const std::string &, double>(&css_set_subspace_weight_by_name),
        .def("setSubspaceWeight",
             &css_set_subspace_weight_by_name,
             py::arg("name"), py::arg("weight"),
             "Set the weight of the subspace with the given name.\n"
             "C++: space.setSubspaceWeight(name, weight);")
        .def("getSubspaces",          &css_get_subspaces,
             "Return the list of all subspaces.\n"
             "C++: space.getSubspaces()")
        .def("getSubspaceWeights",    &css_get_subspace_weights,
             "Return the list of all subspace weights.\n"
             "C++: space.getSubspaceWeights()")
        .def("isLocked",              &css_is_locked,
             "Return True if no more subspaces can be added.\n"
             "C++: space.isLocked()")
        .def("lock",                  &css_lock,
             "Lock the space — prevent adding further subspaces.\n"
             "Called automatically by SE2StateSpace, SE3StateSpace constructors.\n"
             "C++: space.lock();")

        // --- Inherited core ops (overridden in CompoundStateSpace) ---
        .def("getDimension",          [](const CompoundStateSpace &s) { return s.getDimension(); },
             "Return total dimensions across all subspaces.\n"
             "C++: space.getDimension()")
        .def("getMaximumExtent",      [](const CompoundStateSpace &s) { return s.getMaximumExtent(); },
             "Return weighted maximum extent across all subspaces.\n"
             "C++: space.getMaximumExtent()")
        .def("getMeasure",            [](const CompoundStateSpace &s) { return s.getMeasure(); },
             "Return weighted measure (volume) across all subspaces.\n"
             "C++: space.getMeasure()")
        .def("isCompound",            [](const CompoundStateSpace &s) { return s.isCompound(); },
             "Always returns True for CompoundStateSpace.\n"
             "C++: space.isCompound()")
        .def("isHybrid",              [](const CompoundStateSpace &s) { return s.isHybrid(); },
             "Return True if any subspace is discrete.\n"
             "C++: space.isHybrid()")
        .def("setup",                 [](CompoundStateSpace &s) { s.setup(); },
             "Perform setup on this space and all subspaces.\n"
             "C++: space.setup();");
}