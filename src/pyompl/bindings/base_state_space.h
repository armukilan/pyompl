// // #pragma once
// // #include <pybind11/pybind11.h>
// // #include <pybind11/stl.h>

// // #include <ompl/base/State.h>
// // #include <ompl/base/StateSpace.h>
// // #include <ompl/base/spaces/RealVectorStateSpace.h>
// // #include <ompl/base/spaces/RealVectorBounds.h>

// // namespace py = pybind11;
// // using namespace ompl::base;

// // // ---------------------------------------------------------------------------
// // // Helper free functions — MSVC handles these more reliably than inline lambdas
// // // ---------------------------------------------------------------------------

// // static unsigned int rvbounds_get_dimension(const RealVectorBounds &b)
// // {
// //     return static_cast<unsigned int>(b.low.size());
// // }

// // static std::vector<double> rvbounds_get_low(const RealVectorBounds &b)
// // {
// //     return b.low;
// // }

// // static std::vector<double> rvbounds_get_high(const RealVectorBounds &b)
// // {
// //     return b.high;
// // }

// // static double rvstate_getitem(const RealVectorStateSpace::StateType &s, unsigned int i)
// // {
// //     return s[i];
// // }

// // static void rvstate_setitem(RealVectorStateSpace::StateType &s, unsigned int i, double v)
// // {
// //     s[i] = v;
// // }

// // static RealVectorStateSpace::StateType *rvspace_alloc(RealVectorStateSpace &space)
// // {
// //     return space.allocState()->as<RealVectorStateSpace::StateType>();
// // }

// // static void rvspace_free(RealVectorStateSpace &space, RealVectorStateSpace::StateType *st)
// // {
// //     space.freeState(st);
// // }

// // static double rvspace_distance(RealVectorStateSpace &space,
// //                                const RealVectorStateSpace::StateType *s1,
// //                                const RealVectorStateSpace::StateType *s2)
// // {
// //     return space.distance(s1, s2);
// // }

// // static void rvspace_interpolate(RealVectorStateSpace &space,
// //                                 const RealVectorStateSpace::StateType *from,
// //                                 const RealVectorStateSpace::StateType *to,
// //                                 double t,
// //                                 RealVectorStateSpace::StateType *result)
// // {
// //     space.interpolate(from, to, t, result);
// // }

// // static bool rvspace_satisfies_bounds(RealVectorStateSpace &space,
// //                                      const RealVectorStateSpace::StateType *s)
// // {
// //     return space.satisfiesBounds(s);
// // }

// // static void rvspace_enforce_bounds(RealVectorStateSpace &space,
// //                                    RealVectorStateSpace::StateType *s)
// // {
// //     space.enforceBounds(s);
// // }

// // static std::string rvspace_repr(const RealVectorStateSpace &s)
// // {
// //     return "<RealVectorStateSpace dim=" + std::to_string(s.getDimension()) + ">";
// // }

// // // ---------------------------------------------------------------------------
// // // Binding function
// // // ---------------------------------------------------------------------------

// // inline void bind_base_state_space(py::module_ &m)
// // {
// //     // ------------------------------------------------------------------ //
// //     //  StateSpace (abstract base) — MUST be registered first so that
// //     //  RealVectorStateSpace can declare it as a base class in pybind11.
// //     //  Without this, pybind11 throws "referenced unknown base type".
// //     // ------------------------------------------------------------------ //
// //     // py::class_<StateSpace, std::shared_ptr<StateSpace>>(m, "StateSpace",
// //     //     "Abstract base class for all OMPL state spaces.\n\n"
// //     //     "C++ header: ompl/base/StateSpace.h\n"
// //     //     "C++ class:  ompl::base::StateSpace\n"
// //     //     "You do not instantiate this directly — use a concrete subclass\n"
// //     //     "like RealVectorStateSpace, SE2StateSpace, etc.")

// //     //     .def("getDimension",     &StateSpace::getDimension,
// //     //          "Return the number of dimensions in this space.\n"
// //     //          "C++: space->getDimension()")

// //     //     .def("getMaximumExtent", &StateSpace::getMaximumExtent,
// //     //          "Return the maximum distance between any two states.\n"
// //     //          "C++: space->getMaximumExtent()")

// //     //     .def("getName",          &StateSpace::getName,
// //     //          "Return the name of this space.\n"
// //     //          "C++: space->getName()")

// //     //     .def("setName",          &StateSpace::setName, py::arg("name"),
// //     //          "Set a name for this space.\n"
// //     //          "C++: space->setName(name);")

// //     //     .def("satisfiesBounds",  &StateSpace::satisfiesBounds, py::arg("state"),
// //     //          "Return True if the given state satisfies the space bounds.\n"
// //     //          "C++: space->satisfiesBounds(state)")

// //     //     .def("enforceBounds",    &StateSpace::enforceBounds, py::arg("state"),
// //     //          "Clamp the state to lie within bounds.\n"
// //     //          "C++: space->enforceBounds(state);")

// //     //     .def("distance",         &StateSpace::distance,
// //     //          py::arg("s1"), py::arg("s2"),
// //     //          "Return the distance between two states.\n"
// //     //          "C++: space->distance(s1, s2)");

// //     // ------------------------------------------------------------------ //
// //     //  RealVectorBounds
// //     // ------------------------------------------------------------------ //
// //     py::class_<RealVectorBounds>(m, "RealVectorBounds",
// //         "Axis-aligned bounding box for a RealVectorStateSpace.\n\n"
// //         "C++ header: ompl/base/spaces/RealVectorBounds.h\n"
// //         "C++ class:  ompl::base::RealVectorBounds")

// //         .def(py::init<unsigned int>(), py::arg("dim"),
// //              "Create bounds for the given number of dimensions.\n"
// //              "C++: RealVectorBounds bounds(dim);")

// //         .def("setLow",
// //              py::overload_cast<double>(&RealVectorBounds::setLow),
// //              py::arg("value"),
// //              "Set ALL lower bounds to the same value.\n"
// //              "C++: bounds.setLow(value);")

// //         .def("setLow",
// //              py::overload_cast<unsigned int, double>(&RealVectorBounds::setLow),
// //              py::arg("index"), py::arg("value"),
// //              "Set the lower bound for one specific dimension.\n"
// //              "C++: bounds.setLow(index, value);")

// //         .def("setHigh",
// //              py::overload_cast<double>(&RealVectorBounds::setHigh),
// //              py::arg("value"),
// //              "Set ALL upper bounds to the same value.\n"
// //              "C++: bounds.setHigh(value);")

// //         .def("setHigh",
// //              py::overload_cast<unsigned int, double>(&RealVectorBounds::setHigh),
// //              py::arg("index"), py::arg("value"),
// //              "Set the upper bound for one specific dimension.\n"
// //              "C++: bounds.setHigh(index, value);")

// //         .def("getLow",        &rvbounds_get_low,
// //              "Return the list of lower bounds.\n"
// //              "C++: bounds.low  (public std::vector<double>)")

// //         .def("getHigh",       &rvbounds_get_high,
// //              "Return the list of upper bounds.\n"
// //              "C++: bounds.high  (public std::vector<double>)")

// //         .def("check",         &RealVectorBounds::check,
// //              "Validate bounds — throws if low[i] >= high[i].\n"
// //              "C++: bounds.check();")

// //         .def("getDimension",  &rvbounds_get_dimension,
// //              "Return the number of dimensions (size of low/high vectors).\n"
// //              "C++: bounds.low.size()")

// //         .def("getVolume",     &RealVectorBounds::getVolume,
// //              "Return the volume of the bounded region.\n"
// //              "C++: bounds.getVolume()")

// //         .def("getDifference", &RealVectorBounds::getDifference,
// //              "Return high[i] - low[i] for each dimension.\n"
// //              "C++: bounds.getDifference()");

// //     // ------------------------------------------------------------------ //
// //     //  RealVectorStateSpace::StateType
// //     // ------------------------------------------------------------------ //
// //     py::class_<RealVectorStateSpace::StateType>(m, "RealVectorState",
// //         "A single state in a RealVectorStateSpace — an array of doubles.\n\n"
// //         "C++ type: ompl::base::RealVectorStateSpace::StateType\n"
// //         "Do NOT construct directly. Use space.allocState() instead.")

// //         .def("__getitem__", &rvstate_getitem, py::arg("index"),
// //              "Read value at dimension index.\n"
// //              "C++: state->values[index]  or  (*state)[index]")

// //         .def("__setitem__", &rvstate_setitem, py::arg("index"), py::arg("value"),
// //              "Write value at dimension index.\n"
// //              "C++: state->values[index] = value");

// //     // ------------------------------------------------------------------ //
// //     //  RealVectorStateSpace
// //     // ------------------------------------------------------------------ //
// //     py::class_<RealVectorStateSpace, StateSpace,
// //                std::shared_ptr<RealVectorStateSpace>>(
// //         m, "RealVectorStateSpace",
// //         "A state space where each state is a point in R^n.\n\n"
// //         "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
// //         "C++ class:  ompl::base::RealVectorStateSpace\n"
// //         "Typical use: robot with n joints, or a point in n-dimensional space.")

// //         .def(py::init<unsigned int>(), py::arg("dim") = 0,
// //              "Construct with the given number of dimensions.\n"
// //              "C++: auto space = std::make_shared<RealVectorStateSpace>(dim);")

// //         .def("getDimension",     &RealVectorStateSpace::getDimension,
// //              "Return the number of dimensions.\n"
// //              "C++: space->getDimension()")

// //         .def("setBounds",
// //              py::overload_cast<const RealVectorBounds &>(
// //                  &RealVectorStateSpace::setBounds),
// //              py::arg("bounds"),
// //              "Set the bounds for all dimensions.\n"
// //              "C++: space->setBounds(bounds);")

// //         .def("getBounds",        &RealVectorStateSpace::getBounds,
// //              "Return the current bounds.\n"
// //              "C++: space->getBounds()")

// //         .def("getMaximumExtent", &RealVectorStateSpace::getMaximumExtent,
// //              "Return the maximum possible distance between any two states.\n"
// //              "C++: space->getMaximumExtent()")

// //         .def("allocState",       &rvspace_alloc,
// //              py::return_value_policy::reference_internal,
// //              "Allocate and return a new state (memory managed by the space).\n"
// //              "Always pair with freeState() when done.\n"
// //              "C++: space->allocState()->as<RealVectorStateSpace::StateType>()")

// //         .def("freeState",        &rvspace_free,        py::arg("state"),
// //              "Free a previously allocated state.\n"
// //              "C++: space->freeState(state);")

// //         .def("distance",         &rvspace_distance,    py::arg("s1"), py::arg("s2"),
// //              "Return the Euclidean distance between two states.\n"
// //              "C++: space->distance(s1, s2)")

// //         .def("interpolate",      &rvspace_interpolate,
// //              py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
// //              "Interpolate between two states at fraction t in [0,1].\n"
// //              "t=0 -> from,  t=1 -> to,  t=0.5 -> midpoint.\n"
// //              "C++: space->interpolate(from, to, t, result);")

// //         .def("satisfiesBounds",  &rvspace_satisfies_bounds, py::arg("state"),
// //              "Return True if the state is within the set bounds.\n"
// //              "C++: space->satisfiesBounds(state)")

// //         .def("enforceBounds",    &rvspace_enforce_bounds,   py::arg("state"),
// //              "Clamp the state values to lie within bounds (modifies in place).\n"
// //              "C++: space->enforceBounds(state);")

// //         .def("getName",          &RealVectorStateSpace::getName,
// //              "Return the name of this space.\n"
// //              "C++: space->getName()")

// //         .def("setName",          &RealVectorStateSpace::setName, py::arg("name"),
// //              "Set a name for this space (useful for debugging).\n"
// //              "C++: space->setName(name);")

// //         .def("__repr__",         &rvspace_repr);
// // }
















// #pragma once
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>

// #include <ompl/base/State.h>
// #include <ompl/base/StateSpace.h>
// #include <ompl/base/spaces/RealVectorBounds.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/StateSampler.h>

// #include <sstream>

// namespace py = pybind11;
// using namespace ompl::base;

// // ===========================================================================
// // RealVectorBounds free helpers
// // ===========================================================================

// static std::vector<double> rvb_get_low(const RealVectorBounds &b)
// {
//     return b.low;
// }
// static std::vector<double> rvb_get_high(const RealVectorBounds &b)
// {
//     return b.high;
// }
// static unsigned int rvb_get_dimension(const RealVectorBounds &b)
// {
//     return static_cast<unsigned int>(b.low.size());
// }
// static void rvb_set_low_all(RealVectorBounds &b, double v)
// {
//     b.setLow(v);
// }
// static void rvb_set_low_index(RealVectorBounds &b, unsigned int i, double v)
// {
//     b.setLow(i, v);
// }
// static void rvb_set_high_all(RealVectorBounds &b, double v)
// {
//     b.setHigh(v);
// }
// static void rvb_set_high_index(RealVectorBounds &b, unsigned int i, double v)
// {
//     b.setHigh(i, v);
// }
// static void rvb_resize(RealVectorBounds &b, std::size_t size)
// {
//     b.resize(size);
// }
// static double rvb_get_volume(const RealVectorBounds &b)
// {
//     return b.getVolume();
// }
// static std::vector<double> rvb_get_difference(const RealVectorBounds &b)
// {
//     return b.getDifference();
// }
// static void rvb_check(const RealVectorBounds &b)
// {
//     b.check();
// }

// // ===========================================================================
// // RealVectorStateSpace::StateType free helpers
// // ===========================================================================

// static double rvstate_getitem(const RealVectorStateSpace::StateType &s, unsigned int i)
// {
//     return s[i];
// }
// static void rvstate_setitem(RealVectorStateSpace::StateType &s, unsigned int i, double v)
// {
//     s[i] = v;
// }

// // ===========================================================================
// // RealVectorStateSpace free helpers
// // ===========================================================================

// static void rvss_add_dimension_bounds(RealVectorStateSpace &s, double lo, double hi)
// {
//     s.addDimension(lo, hi);
// }
// static void rvss_add_dimension_named(RealVectorStateSpace &s,
//                                      const std::string &name, double lo, double hi)
// {
//     s.addDimension(name, lo, hi);
// }
// static void rvss_set_bounds_obj(RealVectorStateSpace &s, const RealVectorBounds &b)
// {
//     s.setBounds(b);
// }
// static void rvss_set_bounds_scalar(RealVectorStateSpace &s, double lo, double hi)
// {
//     s.setBounds(lo, hi);
// }
// static const RealVectorBounds &rvss_get_bounds(const RealVectorStateSpace &s)
// {
//     return s.getBounds();
// }
// static unsigned int rvss_get_dimension(const RealVectorStateSpace &s)
// {
//     return s.getDimension();
// }
// static const std::string &rvss_get_dimension_name(const RealVectorStateSpace &s, unsigned int i)
// {
//     return s.getDimensionName(i);
// }
// static int rvss_get_dimension_index(const RealVectorStateSpace &s, const std::string &name)
// {
//     return s.getDimensionIndex(name);
// }
// static void rvss_set_dimension_name(RealVectorStateSpace &s, unsigned int i, const std::string &name)
// {
//     s.setDimensionName(i, name);
// }
// static double rvss_get_maximum_extent(const RealVectorStateSpace &s)
// {
//     return s.getMaximumExtent();
// }
// static double rvss_get_measure(const RealVectorStateSpace &s)
// {
//     return s.getMeasure();
// }
// static void rvss_enforce_bounds(const RealVectorStateSpace &s, State *st)
// {
//     s.enforceBounds(st);
// }
// static bool rvss_satisfies_bounds(const RealVectorStateSpace &s, const State *st)
// {
//     return s.satisfiesBounds(st);
// }
// static void rvss_copy_state(const RealVectorStateSpace &s, State *dst, const State *src)
// {
//     s.copyState(dst, src);
// }
// static unsigned int rvss_get_serialization_length(const RealVectorStateSpace &s)
// {
//     return s.getSerializationLength();
// }
// static double rvss_distance(const RealVectorStateSpace &s, const State *s1, const State *s2)
// {
//     return s.distance(s1, s2);
// }
// static bool rvss_equal_states(const RealVectorStateSpace &s, const State *s1, const State *s2)
// {
//     return s.equalStates(s1, s2);
// }
// static void rvss_interpolate(const RealVectorStateSpace &s,
//                              const State *from, const State *to, double t, State *result)
// {
//     s.interpolate(from, to, t, result);
// }
// static RealVectorStateSpace::StateType *rvss_alloc_state(RealVectorStateSpace &s)
// {
//     return s.allocState()->as<RealVectorStateSpace::StateType>();
// }
// static void rvss_free_state(RealVectorStateSpace &s, RealVectorStateSpace::StateType *st)
// {
//     s.freeState(st);
// }
// static StateSamplerPtr rvss_alloc_default_sampler(const RealVectorStateSpace &s)
// {
//     return s.allocDefaultStateSampler();
// }
// static std::string rvss_print_state(const RealVectorStateSpace &s, const State *st)
// {
//     std::ostringstream oss;
//     s.printState(st, oss);
//     return oss.str();
// }
// static std::string rvss_print_settings(const RealVectorStateSpace &s)
// {
//     std::ostringstream oss;
//     s.printSettings(oss);
//     return oss.str();
// }
// static void rvss_register_projections(RealVectorStateSpace &s)
// {
//     s.registerProjections();
// }
// static void rvss_setup(RealVectorStateSpace &s)
// {
//     s.setup();
// }
// static std::string rvss_repr(const RealVectorStateSpace &s)
// {
//     return "<RealVectorStateSpace dim=" + std::to_string(s.getDimension()) + ">";
// }

// // ===========================================================================
// // Binding function
// // ===========================================================================

// inline void bind_base_state_space(py::module_ &m)
// {
//     // -------------------------------------------------------------------
//     // RealVectorBounds
//     // -------------------------------------------------------------------
//     py::class_<RealVectorBounds>(m, "RealVectorBounds",
//         "Axis-aligned bounding box for a RealVectorStateSpace.\n\n"
//         "C++ header: ompl/base/spaces/RealVectorBounds.h\n"
//         "C++ class:  ompl::base::RealVectorBounds\n\n"
//         "Stores a lower and upper bound for each dimension.\n"
//         "Must call setBounds() on the space after configuring this.")

//         .def(py::init<unsigned int>(), py::arg("dim"),
//              "Create bounds for the given number of dimensions.\n"
//              "Initialises low[] and high[] vectors of size dim.\n"
//              "C++: RealVectorBounds bounds(dim);")

//         // --- setLow ---
//         .def("setLow", &rvb_set_low_all, py::arg("value"),
//              "Set ALL lower bounds to the same value.\n"
//              "C++: bounds.setLow(value);")
//         .def("setLow", &rvb_set_low_index, py::arg("index"), py::arg("value"),
//              "Set the lower bound for one specific dimension.\n"
//              "C++: bounds.setLow(index, value);")

//         // --- setHigh ---
//         .def("setHigh", &rvb_set_high_all, py::arg("value"),
//              "Set ALL upper bounds to the same value.\n"
//              "C++: bounds.setHigh(value);")
//         .def("setHigh", &rvb_set_high_index, py::arg("index"), py::arg("value"),
//              "Set the upper bound for one specific dimension.\n"
//              "C++: bounds.setHigh(index, value);")

//         // --- resize ---
//         .def("resize", &rvb_resize, py::arg("size"),
//              "Resize the bounds to a new number of dimensions.\n"
//              "New entries are zero-initialised.\n"
//              "C++: bounds.resize(size);")

//         // --- accessors ---
//         .def("getLow",  &rvb_get_low,
//              "Return a copy of the lower bounds as a Python list.\n"
//              "C++: bounds.low  (public std::vector<double>)")
//         .def("getHigh", &rvb_get_high,
//              "Return a copy of the upper bounds as a Python list.\n"
//              "C++: bounds.high  (public std::vector<double>)")
//         .def("getDimension", &rvb_get_dimension,
//              "Return the number of dimensions (= len(low) = len(high)).\n"
//              "C++: bounds.low.size()")

//         // --- computed properties ---
//         .def("getVolume", &rvb_get_volume,
//              "Return the volume of the bounded region.\n"
//              "= product of (high[i] - low[i]) for all i.\n"
//              "C++: bounds.getVolume()")
//         .def("getDifference", &rvb_get_difference,
//              "Return high[i] - low[i] for each dimension as a list.\n"
//              "C++: bounds.getDifference()")

//         // --- validation ---
//         .def("check", &rvb_check,
//              "Validate bounds. Throws if low[i] >= high[i] for any i,\n"
//              "or if low and high have different lengths.\n"
//              "C++: bounds.check();");

//     // -------------------------------------------------------------------
//     // RealVectorStateSampler
//     // -------------------------------------------------------------------
//     py::class_<RealVectorStateSampler, StateSampler,
//                std::shared_ptr<RealVectorStateSampler>>(m, "RealVectorStateSampler",
//         "Sampler for the RealVectorStateSpace.\n\n"
//         "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
//         "C++ class:  ompl::base::RealVectorStateSampler\n\n"
//         "Do not instantiate directly.\n"
//         "Use space.allocDefaultStateSampler() to get one.")

//         .def("sampleUniform",
//              [](RealVectorStateSampler &s, State *st) { s.sampleUniform(st); },
//              py::arg("state"),
//              "Sample a state uniformly at random within the space bounds.\n"
//              "C++: sampler->sampleUniform(state);")

//         .def("sampleUniformNear",
//              [](RealVectorStateSampler &s, State *st, const State *near, double dist) {
//                  s.sampleUniformNear(st, near, dist);
//              },
//              py::arg("state"), py::arg("near"), py::arg("distance"),
//              "Sample a state uniformly from [near[i]-distance, near[i]+distance].\n"
//              "Interval is clamped to the space bounds.\n"
//              "C++: sampler->sampleUniformNear(state, near, distance);")

//         .def("sampleGaussian",
//              [](RealVectorStateSampler &s, State *st, const State *mean, double stddev) {
//                  s.sampleGaussian(st, mean, stddev);
//              },
//              py::arg("state"), py::arg("mean"), py::arg("stdDev"),
//              "Sample a state from a Gaussian centred at 'mean' with given stdDev.\n"
//              "Values outside bounds are clamped to the nearest boundary.\n"
//              "C++: sampler->sampleGaussian(state, mean, stdDev);");

//     // -------------------------------------------------------------------
//     // RealVectorStateSpace::StateType
//     // -------------------------------------------------------------------
//     py::class_<RealVectorStateSpace::StateType>(m, "RealVectorState",
//         "A single state in a RealVectorStateSpace — an array of doubles.\n\n"
//         "C++ type: ompl::base::RealVectorStateSpace::StateType\n\n"
//         "Do NOT construct directly.\n"
//         "Use space.allocState() to get one, space.freeState() when done.\n"
//         "Access values with indexing: state[0], state[1], ...")

//         .def("__getitem__", &rvstate_getitem, py::arg("index"),
//              "Read the value at dimension 'index'.\n"
//              "C++: state->values[index]  or  (*state)[index]")
//         .def("__setitem__", &rvstate_setitem, py::arg("index"), py::arg("value"),
//              "Write a value at dimension 'index'.\n"
//              "C++: state->values[index] = value");

//     // -------------------------------------------------------------------
//     // RealVectorStateSpace
//     // -------------------------------------------------------------------
//     py::class_<RealVectorStateSpace, StateSpace,
//                std::shared_ptr<RealVectorStateSpace>>(m, "RealVectorStateSpace",
//         "A state space representing R^n (n-dimensional Euclidean space).\n\n"
//         "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
//         "C++ class:  ompl::base::RealVectorStateSpace\n\n"
//         "Use this when your robot's configuration is described by n real numbers.\n"
//         "Examples:\n"
//         "  - A point robot in 2D: dim=2, bounds = workspace limits\n"
//         "  - A robot arm with 6 joints: dim=6, bounds = joint angle limits\n"
//         "  - A drone in 3D: dim=3, bounds = flight envelope\n\n"
//         "Distance metric: Euclidean (L2 norm).\n"
//         "Always call setBounds() before using the space.")

//         .def(py::init<unsigned int>(), py::arg("dim") = 0,
//              "Construct with the given number of dimensions.\n"
//              "C++: auto space = std::make_shared<RealVectorStateSpace>(dim);")

//         // --- dimension management ---
//         .def("addDimension", &rvss_add_dimension_bounds,
//              py::arg("minBound") = 0.0, py::arg("maxBound") = 0.0,
//              "Add one more dimension to the space, with optional bounds.\n"
//              "You must call setup() after adding dimensions.\n"
//              "C++: space->addDimension(minBound, maxBound);")
//         .def("addDimension", &rvss_add_dimension_named,
//              py::arg("name"), py::arg("minBound") = 0.0, py::arg("maxBound") = 0.0,
//              "Add one more named dimension to the space, with optional bounds.\n"
//              "You must call setup() after adding dimensions.\n"
//              "C++: space->addDimension(name, minBound, maxBound);")

//         // --- bounds ---
//         .def("setBounds", &rvss_set_bounds_obj, py::arg("bounds"),
//              "Set bounds from a RealVectorBounds object.\n"
//              "C++: space->setBounds(bounds);")
//         .def("setBounds", &rvss_set_bounds_scalar, py::arg("low"), py::arg("high"),
//              "Set the same [low, high] bounds for ALL dimensions.\n"
//              "C++: space->setBounds(low, high);")
//         .def("getBounds", &rvss_get_bounds,
//              py::return_value_policy::reference_internal,
//              "Return the current bounds.\n"
//              "C++: space->getBounds()")

//         // --- dimension info ---
//         .def("getDimension",     &rvss_get_dimension,
//              "Return the number of dimensions.\n"
//              "C++: space->getDimension()")
//         .def("getDimensionName", &rvss_get_dimension_name, py::arg("index"),
//              "Return the name of dimension 'index'. Empty string if unnamed.\n"
//              "C++: space->getDimensionName(index)")
//         .def("getDimensionIndex", &rvss_get_dimension_index, py::arg("name"),
//              "Return the index of the dimension with the given name.\n"
//              "Returns -1 if the name is not found.\n"
//              "C++: space->getDimensionIndex(name)")
//         .def("setDimensionName", &rvss_set_dimension_name,
//              py::arg("index"), py::arg("name"),
//              "Set the name of dimension 'index'.\n"
//              "C++: space->setDimensionName(index, name);")

//         // --- extent and measure ---
//         .def("getMaximumExtent", &rvss_get_maximum_extent,
//              "Return the maximum possible distance between any two states.\n"
//              "= L2 norm of (high - low) across all dimensions.\n"
//              "C++: space->getMaximumExtent()")
//         .def("getMeasure", &rvss_get_measure,
//              "Return the volume (measure) of the bounded region.\n"
//              "= product of (high[i] - low[i]) for all i.\n"
//              "C++: space->getMeasure()")

//         // --- bounds checking ---
//         .def("enforceBounds", &rvss_enforce_bounds, py::arg("state"),
//              "Clamp all state values to lie within bounds (modifies in place).\n"
//              "C++: space->enforceBounds(state);")
//         .def("satisfiesBounds", &rvss_satisfies_bounds, py::arg("state"),
//              "Return True if all state values are within the bounds.\n"
//              "C++: space->satisfiesBounds(state)")

//         // --- state copy ---
//         .def("copyState", &rvss_copy_state,
//              py::arg("destination"), py::arg("source"),
//              "Copy source state into destination.\n"
//              "Memory of source and destination must not overlap.\n"
//              "C++: space->copyState(destination, source);")

//         // --- serialization ---
//         .def("getSerializationLength", &rvss_get_serialization_length,
//              "Return the number of bytes needed to serialise one state.\n"
//              "= dim * sizeof(double)\n"
//              "C++: space->getSerializationLength()")

//         // --- distance and equality ---
//         .def("distance", &rvss_distance, py::arg("state1"), py::arg("state2"),
//              "Return the Euclidean (L2) distance between two states.\n"
//              "C++: space->distance(state1, state2)")
//         .def("equalStates", &rvss_equal_states, py::arg("state1"), py::arg("state2"),
//              "Return True if two states are exactly equal.\n"
//              "C++: space->equalStates(state1, state2)")

//         // --- interpolation ---
//         .def("interpolate", &rvss_interpolate,
//              py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
//              "Compute the state at fraction t in [0,1] between 'from' and 'to'.\n"
//              "t=0 -> from,  t=1 -> to,  t=0.5 -> midpoint.\n"
//              "Result is stored in 'result' (must be pre-allocated).\n"
//              "C++: space->interpolate(from, to, t, result);")

//         // --- state allocation ---
//         .def("allocState", &rvss_alloc_state,
//              py::return_value_policy::reference_internal,
//              "Allocate a new state. Memory is managed by the space.\n"
//              "Always call freeState() when you are done with it.\n"
//              "C++: space->allocState()->as<RealVectorStateSpace::StateType>()")
//         .def("freeState", &rvss_free_state, py::arg("state"),
//              "Free a state previously allocated by allocState().\n"
//              "C++: space->freeState(state);")

//         // --- sampler ---
//         .def("allocDefaultStateSampler", &rvss_alloc_default_sampler,
//              "Allocate the default uniform sampler for this space.\n"
//              "Returns a RealVectorStateSampler.\n"
//              "C++: space->allocDefaultStateSampler()")

//         // --- debug / print ---
//         .def("printState", &rvss_print_state, py::arg("state"),
//              "Return a string representation of the given state.\n"
//              "C++: space->printState(state, std::cout);")
//         .def("printSettings", &rvss_print_settings,
//              "Return a string describing the space settings (dim, bounds, etc.).\n"
//              "C++: space->printSettings(std::cout);")

//         // --- projections and setup ---
//         .def("registerProjections", &rvss_register_projections,
//              "Register the default projection for this space.\n"
//              "Called automatically by setup().\n"
//              "C++: space->registerProjections();")
//         .def("setup", &rvss_setup,
//              "Finalise the space. Call after all dimensions and bounds are set.\n"
//              "Also called automatically by SpaceInformation.\n"
//              "C++: space->setup();")

//         // --- name (inherited but commonly used) ---
//         .def("getName", &RealVectorStateSpace::getName,
//              "Return the name of this space.\n"
//              "C++: space->getName()")
//         .def("setName", &RealVectorStateSpace::setName, py::arg("name"),
//              "Set the name of this space.\n"
//              "C++: space->setName(name);")

//         .def("__repr__", &rvss_repr);
// }


#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSampler.h>

#include <sstream>

namespace py = pybind11;
using namespace ompl::base;

// Convenience alias
using RVST = RealVectorStateSpace::StateType;

// ===========================================================================
// RealVectorBounds free helpers
// ===========================================================================

static std::vector<double> rvb_get_low(const RealVectorBounds &b)   { return b.low; }
static std::vector<double> rvb_get_high(const RealVectorBounds &b)  { return b.high; }
static unsigned int rvb_get_dimension(const RealVectorBounds &b)
{
    return static_cast<unsigned int>(b.low.size());
}
static void rvb_set_low_all(RealVectorBounds &b, double v)                      { b.setLow(v); }
static void rvb_set_low_index(RealVectorBounds &b, unsigned int i, double v)    { b.setLow(i, v); }
static void rvb_set_high_all(RealVectorBounds &b, double v)                     { b.setHigh(v); }
static void rvb_set_high_index(RealVectorBounds &b, unsigned int i, double v)   { b.setHigh(i, v); }
static void rvb_resize(RealVectorBounds &b, std::size_t size)                   { b.resize(size); }
static double rvb_get_volume(const RealVectorBounds &b)                         { return b.getVolume(); }
static std::vector<double> rvb_get_difference(const RealVectorBounds &b)        { return b.getDifference(); }
static void rvb_check(const RealVectorBounds &b)                                { b.check(); }

// ===========================================================================
// RealVectorStateSpace::StateType free helpers
// ===========================================================================

static double rvstate_getitem(const RVST &s, unsigned int i) { return s[i]; }
static void   rvstate_setitem(RVST &s, unsigned int i, double v) { s[i] = v; }

// ===========================================================================
// RealVectorStateSpace free helpers
// NOTE: All state arguments use RVST* (RealVectorState from Python) and are
//       cast to State* when calling OMPL. This avoids the TypeError that
//       occurs when Python passes RealVectorState where State* is expected.
// ===========================================================================

static void rvss_add_dimension_bounds(RealVectorStateSpace &s, double lo, double hi)
{
    s.addDimension(lo, hi);
}
static void rvss_add_dimension_named(RealVectorStateSpace &s,
                                     const std::string &name, double lo, double hi)
{
    s.addDimension(name, lo, hi);
}
static void rvss_set_bounds_obj(RealVectorStateSpace &s, const RealVectorBounds &b)
{
    s.setBounds(b);
}
static void rvss_set_bounds_scalar(RealVectorStateSpace &s, double lo, double hi)
{
    s.setBounds(lo, hi);
}
static const RealVectorBounds &rvss_get_bounds(const RealVectorStateSpace &s)
{
    return s.getBounds();
}
static unsigned int rvss_get_dimension(const RealVectorStateSpace &s)
{
    return s.getDimension();
}
static const std::string &rvss_get_dimension_name(const RealVectorStateSpace &s, unsigned int i)
{
    return s.getDimensionName(i);
}
static int rvss_get_dimension_index(const RealVectorStateSpace &s, const std::string &name)
{
    return s.getDimensionIndex(name);
}
static void rvss_set_dimension_name(RealVectorStateSpace &s, unsigned int i, const std::string &name)
{
    s.setDimensionName(i, name);
}
static double rvss_get_maximum_extent(const RealVectorStateSpace &s) { return s.getMaximumExtent(); }
static double rvss_get_measure(const RealVectorStateSpace &s)        { return s.getMeasure(); }

// --- State operations: accept RVST* and cast to State* ---

static void rvss_enforce_bounds(const RealVectorStateSpace &s, RVST *st)
{
    s.enforceBounds(static_cast<State *>(st));
}
static bool rvss_satisfies_bounds(const RealVectorStateSpace &s, const RVST *st)
{
    return s.satisfiesBounds(static_cast<const State *>(st));
}
static void rvss_copy_state(const RealVectorStateSpace &s, RVST *dst, const RVST *src)
{
    s.copyState(static_cast<State *>(dst), static_cast<const State *>(src));
}
static unsigned int rvss_get_serialization_length(const RealVectorStateSpace &s)
{
    return s.getSerializationLength();
}
static double rvss_distance(const RealVectorStateSpace &s, const RVST *s1, const RVST *s2)
{
    return s.distance(static_cast<const State *>(s1), static_cast<const State *>(s2));
}
static bool rvss_equal_states(const RealVectorStateSpace &s, const RVST *s1, const RVST *s2)
{
    return s.equalStates(static_cast<const State *>(s1), static_cast<const State *>(s2));
}
static void rvss_interpolate(const RealVectorStateSpace &s,
                             const RVST *from, const RVST *to, double t, RVST *result)
{
    s.interpolate(static_cast<const State *>(from),
                  static_cast<const State *>(to),
                  t,
                  static_cast<State *>(result));
}
static RVST *rvss_alloc_state(RealVectorStateSpace &s)
{
    return s.allocState()->as<RVST>();
}
static void rvss_free_state(RealVectorStateSpace &s, RVST *st)
{
    s.freeState(static_cast<State *>(st));
}
static RVST *rvss_clone_state(const RealVectorStateSpace &s, const RVST *src)
{
    return s.cloneState(static_cast<const State *>(src))->as<RVST>();
}
static StateSamplerPtr rvss_alloc_default_sampler(const RealVectorStateSpace &s)
{
    return s.allocDefaultStateSampler();
}
static StateSamplerPtr rvss_alloc_state_sampler(const RealVectorStateSpace &s)
{
    return s.allocStateSampler();
}
static std::string rvss_print_state(const RealVectorStateSpace &s, const RVST *st)
{
    std::ostringstream oss;
    s.printState(static_cast<const State *>(st), oss);
    return oss.str();
}
static std::string rvss_print_settings(const RealVectorStateSpace &s)
{
    std::ostringstream oss;
    s.printSettings(oss);
    return oss.str();
}
static void rvss_register_projections(RealVectorStateSpace &s) { s.registerProjections(); }
static void rvss_setup(RealVectorStateSpace &s)                { s.setup(); }
static std::string rvss_repr(const RealVectorStateSpace &s)
{
    return "<RealVectorStateSpace dim=" + std::to_string(s.getDimension()) + ">";
}

// copyToReals / copyFromReals via base class (work fine with cast)
static std::vector<double> rvss_copy_to_reals(const RealVectorStateSpace &s, const RVST *st)
{
    std::vector<double> reals;
    s.copyToReals(reals, static_cast<const State *>(st));
    return reals;
}
static void rvss_copy_from_reals(const RealVectorStateSpace &s,
                                  RVST *dst, const std::vector<double> &reals)
{
    s.copyFromReals(static_cast<State *>(dst), reals);
}

// ===========================================================================
// Binding function
// ===========================================================================

inline void bind_base_state_space(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // RealVectorBounds
    // -----------------------------------------------------------------------
    py::class_<RealVectorBounds>(m, "RealVectorBounds",
        "Axis-aligned bounding box for a RealVectorStateSpace.\n\n"
        "C++ header: ompl/base/spaces/RealVectorBounds.h\n"
        "C++ class:  ompl::base::RealVectorBounds\n\n"
        "Stores a lower (low[]) and upper (high[]) bound for each dimension.\n"
        "Must be passed to space.setBounds() to take effect.")

        .def(py::init<unsigned int>(), py::arg("dim"),
             "Create bounds for the given number of dimensions.\n"
             "Initialises low[] and high[] vectors of size dim.\n"
             "C++: RealVectorBounds bounds(dim);")

        .def("setLow",  &rvb_set_low_all,   py::arg("value"),
             "Set ALL lower bounds to the same value.\n"
             "C++: bounds.setLow(value);")
        .def("setLow",  &rvb_set_low_index, py::arg("index"), py::arg("value"),
             "Set the lower bound for one specific dimension.\n"
             "C++: bounds.setLow(index, value);")

        .def("setHigh", &rvb_set_high_all,   py::arg("value"),
             "Set ALL upper bounds to the same value.\n"
             "C++: bounds.setHigh(value);")
        .def("setHigh", &rvb_set_high_index, py::arg("index"), py::arg("value"),
             "Set the upper bound for one specific dimension.\n"
             "C++: bounds.setHigh(index, value);")

        .def("resize",  &rvb_resize, py::arg("size"),
             "Resize bounds to a new number of dimensions.\n"
             "New entries are zero-initialised.\n"
             "C++: bounds.resize(size);")

        .def("getLow",        &rvb_get_low,
             "Return a copy of the lower bounds as a Python list.\n"
             "C++: bounds.low  (public std::vector<double>)")
        .def("getHigh",       &rvb_get_high,
             "Return a copy of the upper bounds as a Python list.\n"
             "C++: bounds.high  (public std::vector<double>)")
        .def("getDimension",  &rvb_get_dimension,
             "Return the number of dimensions.\n"
             "C++: bounds.low.size()")
        .def("getVolume",     &rvb_get_volume,
             "Return the volume of the bounded region.\n"
             "= product of (high[i] - low[i]) for all i.\n"
             "C++: bounds.getVolume()")
        .def("getDifference", &rvb_get_difference,
             "Return high[i] - low[i] for each dimension as a list.\n"
             "C++: bounds.getDifference()")
        .def("check",         &rvb_check,
             "Validate bounds. Throws if low[i] >= high[i] for any i.\n"
             "C++: bounds.check();");

    // -----------------------------------------------------------------------
    // RealVectorStateSampler
    // -----------------------------------------------------------------------
    py::class_<RealVectorStateSampler, StateSampler,
               std::shared_ptr<RealVectorStateSampler>>(m, "RealVectorStateSampler",
        "Sampler for the RealVectorStateSpace.\n\n"
        "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
        "C++ class:  ompl::base::RealVectorStateSampler\n\n"
        "Do not construct directly.\n"
        "Use space.allocDefaultStateSampler() to get one.")

        .def("sampleUniform",
             [](RealVectorStateSampler &s, RVST *st) {
                 s.sampleUniform(static_cast<State *>(st));
             },
             py::arg("state"),
             "Sample a state uniformly at random within the space bounds.\n"
             "C++: sampler->sampleUniform(state);")

        .def("sampleUniformNear",
             [](RealVectorStateSampler &s, RVST *st, const RVST *near, double dist) {
                 s.sampleUniformNear(static_cast<State *>(st),
                                     static_cast<const State *>(near), dist);
             },
             py::arg("state"), py::arg("near"), py::arg("distance"),
             "Sample uniformly from [near[i]-distance, near[i]+distance].\n"
             "Clamped to space bounds.\n"
             "C++: sampler->sampleUniformNear(state, near, distance);")

        .def("sampleGaussian",
             [](RealVectorStateSampler &s, RVST *st, const RVST *mean, double stddev) {
                 s.sampleGaussian(static_cast<State *>(st),
                                  static_cast<const State *>(mean), stddev);
             },
             py::arg("state"), py::arg("mean"), py::arg("stdDev"),
             "Sample from a Gaussian centred at 'mean'. Values outside bounds are clamped.\n"
             "C++: sampler->sampleGaussian(state, mean, stdDev);");

    // -----------------------------------------------------------------------
    // RealVectorStateSpace::StateType
    // -----------------------------------------------------------------------
    py::class_<RVST>(m, "RealVectorState",
        "A single state in a RealVectorStateSpace — an array of doubles.\n\n"
        "C++ type: ompl::base::RealVectorStateSpace::StateType\n\n"
        "Do NOT construct directly.\n"
        "Use space.allocState() to get one, space.freeState() when done.\n"
        "Access values with indexing: state[0], state[1], ...")

        .def("__getitem__", &rvstate_getitem, py::arg("index"),
             "Read the value at dimension 'index'.\n"
             "C++: state->values[index]  or  (*state)[index]")
        .def("__setitem__", &rvstate_setitem, py::arg("index"), py::arg("value"),
             "Write a value at dimension 'index'.\n"
             "C++: state->values[index] = value");

    // -----------------------------------------------------------------------
    // RealVectorStateSpace
    // -----------------------------------------------------------------------
    py::class_<RealVectorStateSpace, StateSpace,
               std::shared_ptr<RealVectorStateSpace>>(m, "RealVectorStateSpace",
        "A state space representing R^n (n-dimensional Euclidean space).\n\n"
        "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
        "C++ class:  ompl::base::RealVectorStateSpace\n\n"
        "Use this when your robot config is described by n real numbers.\n"
        "Distance metric: Euclidean (L2 norm).\n"
        "Always call setBounds() before using the space.")

        .def(py::init<unsigned int>(), py::arg("dim") = 0,
             "Construct with the given number of dimensions.\n"
             "C++: auto space = std::make_shared<RealVectorStateSpace>(dim);")

        // dimension management
        .def("addDimension", &rvss_add_dimension_bounds,
             py::arg("minBound") = 0.0, py::arg("maxBound") = 0.0,
             "Add one more dimension with optional bounds.\n"
             "Call setup() after adding dimensions.\n"
             "C++: space->addDimension(minBound, maxBound);")
        .def("addDimension", &rvss_add_dimension_named,
             py::arg("name"), py::arg("minBound") = 0.0, py::arg("maxBound") = 0.0,
             "Add one more named dimension with optional bounds.\n"
             "Call setup() after adding dimensions.\n"
             "C++: space->addDimension(name, minBound, maxBound);")

        // bounds
        .def("setBounds", &rvss_set_bounds_obj, py::arg("bounds"),
             "Set bounds from a RealVectorBounds object.\n"
             "C++: space->setBounds(bounds);")
        .def("setBounds", &rvss_set_bounds_scalar, py::arg("low"), py::arg("high"),
             "Set the same [low, high] bounds for ALL dimensions.\n"
             "C++: space->setBounds(low, high);")
        .def("getBounds", &rvss_get_bounds,
             py::return_value_policy::reference_internal,
             "Return the current bounds.\n"
             "C++: space->getBounds()")

        // dimension info
        .def("getDimension",      &rvss_get_dimension,
             "Return the number of dimensions.\n"
             "C++: space->getDimension()")
        .def("getDimensionName",  &rvss_get_dimension_name,  py::arg("index"),
             "Return the name of dimension 'index'. Empty string if unnamed.\n"
             "C++: space->getDimensionName(index)")
        .def("getDimensionIndex", &rvss_get_dimension_index, py::arg("name"),
             "Return the index of the named dimension. Returns -1 if not found.\n"
             "C++: space->getDimensionIndex(name)")
        .def("setDimensionName",  &rvss_set_dimension_name,
             py::arg("index"), py::arg("name"),
             "Set the name of dimension 'index'.\n"
             "C++: space->setDimensionName(index, name);")

        // extent and measure
        .def("getMaximumExtent",  &rvss_get_maximum_extent,
             "Return the maximum possible distance between any two states.\n"
             "= L2 norm of the diagonal of the bounding box.\n"
             "C++: space->getMaximumExtent()")
        .def("getMeasure",        &rvss_get_measure,
             "Return the volume of the bounded region.\n"
             "= product of (high[i] - low[i]) for all i.\n"
             "C++: space->getMeasure()")

        // bounds checking
        .def("satisfiesBounds",  &rvss_satisfies_bounds, py::arg("state"),
             "Return True if all state values are within the bounds.\n"
             "C++: space->satisfiesBounds(state)")
        .def("enforceBounds",    &rvss_enforce_bounds,   py::arg("state"),
             "Clamp all state values to lie within bounds (modifies in place).\n"
             "C++: space->enforceBounds(state);")

        // state copy and clone
        .def("copyState",  &rvss_copy_state,
             py::arg("destination"), py::arg("source"),
             "Copy source state into destination. Memory must not overlap.\n"
             "C++: space->copyState(destination, source);")
        .def("cloneState", &rvss_clone_state, py::arg("source"),
             py::return_value_policy::reference_internal,
             "Allocate a new state and copy source into it.\n"
             "Must be freed with freeState().\n"
             "C++: space->cloneState(source)->as<RealVectorStateSpace::StateType>()")

        // real value access
        .def("copyToReals",   &rvss_copy_to_reals, py::arg("source"),
             "Return all double values in the state as a Python list.\n"
             "C++: space->copyToReals(reals, source);")
        .def("copyFromReals", &rvss_copy_from_reals,
             py::arg("destination"), py::arg("reals"),
             "Set all state values from a Python list of doubles.\n"
             "C++: space->copyFromReals(destination, reals);")

        // serialization
        .def("getSerializationLength", &rvss_get_serialization_length,
             "Return the number of bytes to serialise one state.\n"
             "= dim * sizeof(double)\n"
             "C++: space->getSerializationLength()")

        // distance and equality
        .def("distance",    &rvss_distance,    py::arg("state1"), py::arg("state2"),
             "Return the Euclidean (L2) distance between two states.\n"
             "C++: space->distance(state1, state2)")
        .def("equalStates", &rvss_equal_states, py::arg("state1"), py::arg("state2"),
             "Return True if two states are exactly equal.\n"
             "C++: space->equalStates(state1, state2)")

        // interpolation
        .def("interpolate", &rvss_interpolate,
             py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
             "Compute the state at fraction t in [0,1] between 'from' and 'to'.\n"
             "t=0 -> from,  t=1 -> to,  t=0.5 -> midpoint.\n"
             "Result stored in 'result' (must be pre-allocated).\n"
             "C++: space->interpolate(from, to, t, result);")

        // state allocation
        .def("allocState",  &rvss_alloc_state,
             py::return_value_policy::reference_internal,
             "Allocate a new state. Memory managed by the space.\n"
             "Always call freeState() when done.\n"
             "C++: space->allocState()->as<RealVectorStateSpace::StateType>()")
        .def("freeState",   &rvss_free_state, py::arg("state"),
             "Free a state previously allocated by allocState().\n"
             "C++: space->freeState(state);")

        // samplers
        .def("allocDefaultStateSampler", &rvss_alloc_default_sampler,
             "Allocate the default uniform sampler for this space.\n"
             "C++: space->allocDefaultStateSampler()")
        .def("allocStateSampler",        &rvss_alloc_state_sampler,
             "Allocate a sampler (custom if set via setStateSamplerAllocator, else default).\n"
             "C++: space->allocStateSampler()")

        // debug / print
        .def("printState",    &rvss_print_state,    py::arg("state"),
             "Return a string representation of the given state.\n"
             "C++: space->printState(state, std::cout);")
        .def("printSettings", &rvss_print_settings,
             "Return a string describing the space settings.\n"
             "C++: space->printSettings(std::cout);")

        // projections and setup
        .def("registerProjections", &rvss_register_projections,
             "Register the default projection. Called automatically by setup().\n"
             "C++: space->registerProjections();")
        .def("setup", &rvss_setup,
             "Finalise the space. Call after all dimensions and bounds are set.\n"
             "Also called automatically by SpaceInformation.\n"
             "C++: space->setup();")

        // name
        .def("getName", &RealVectorStateSpace::getName,
             "Return the name of this space.\n"
             "C++: space->getName()")
        .def("setName", &RealVectorStateSpace::setName, py::arg("name"),
             "Set the name of this space.\n"
             "C++: space->setName(name);")

        .def("__repr__", &rvss_repr);
}