// // #pragma once
// // #include <pybind11/pybind11.h>
// // #include <pybind11/stl.h>
// // #include <pybind11/functional.h>

// // #include <ompl/base/State.h>
// // #include <ompl/base/StateSpace.h>
// // #include <ompl/base/spaces/RealVectorStateSpace.h>
// // #include <ompl/base/spaces/RealVectorBounds.h>

// // namespace py = pybind11;
// // using namespace ompl::base;

// // inline void bind_base_state_space(py::module_ &m) {

// //     // ------------------------------------------------------------------ //
// //     //  RealVectorBounds
// //     // ------------------------------------------------------------------ //
// //     py::class_<RealVectorBounds>(m, "RealVectorBounds",
// //         "Axis-aligned bounding box for a RealVectorStateSpace.")
// //         .def(py::init<unsigned int>(), py::arg("dim"),
// //              "Create bounds for a given number of dimensions.")
// //         .def("setLow",
// //              py::overload_cast<double>(&RealVectorBounds::setLow),
// //              py::arg("value"),
// //              "Set all lower bounds to the same value.")
// //         .def("setLow",
// //              py::overload_cast<unsigned int, double>(&RealVectorBounds::setLow),
// //              py::arg("index"), py::arg("value"),
// //              "Set the lower bound for one dimension.")
// //         .def("setHigh",
// //              py::overload_cast<double>(&RealVectorBounds::setHigh),
// //              py::arg("value"),
// //              "Set all upper bounds to the same value.")
// //         .def("setHigh",
// //              py::overload_cast<unsigned int, double>(&RealVectorBounds::setHigh),
// //              py::arg("index"), py::arg("value"),
// //              "Set the upper bound for one dimension.")
// //         .def("getLow",  [](const RealVectorBounds &b) { return b.low; },
// //              "Return the list of lower bounds.")
// //         .def("getHigh", [](const RealVectorBounds &b) { return b.high; },
// //              "Return the list of upper bounds.")
// //         .def("check", &RealVectorBounds::check,
// //              "Throw if the bounds are invalid (low >= high).")
// //         .def("getDimension", &RealVectorBounds::getDimension,
// //              "Return the number of dimensions.");

// //     // ------------------------------------------------------------------ //
// //     //  RealVectorStateSpace::StateType  (the actual state object)
// //     // ------------------------------------------------------------------ //
// //     py::class_<RealVectorStateSpace::StateType>(m, "RealVectorState",
// //         "A state in a RealVectorStateSpace — essentially a double array.")
// //         .def("__getitem__",
// //              [](const RealVectorStateSpace::StateType &s, unsigned int i) {
// //                  return s[i];
// //              }, py::arg("index"))
// //         .def("__setitem__",
// //              [](RealVectorStateSpace::StateType &s, unsigned int i, double v) {
// //                  s[i] = v;
// //              }, py::arg("index"), py::arg("value"))
// //         .def("__repr__",
// //              [](const RealVectorStateSpace::StateType &s) {
// //                  // We don't know dim here, so expose raw pointer value
// //                  return std::string("<RealVectorState>");
// //              });

// //     // ------------------------------------------------------------------ //
// //     //  RealVectorStateSpace
// //     // ------------------------------------------------------------------ //
// //     py::class_<RealVectorStateSpace, StateSpace,
// //                std::shared_ptr<RealVectorStateSpace>>(
// //         m, "RealVectorStateSpace",
// //         "A state space where each state is a point in R^n.")

// //         .def(py::init<unsigned int>(), py::arg("dim") = 0,
// //              "Construct with the given number of dimensions.")

// //         // .def("getDimension",
// //         //      &RealVectorStateSpace::getDimension,
// //         //      "Return the number of dimensions.")

// //         .def("getDimension", [](const RealVectorBounds &b) {
// //                  return (unsigned int)b.low.size();
// //              },
// //              "Return the number of dimensions.")
// //         .def("getVolume", &RealVectorBounds::getVolume,
// //              "Return the volume of the space enclosed by the bounds.")
// //         .def("getDifference", &RealVectorBounds::getDifference,
// //              "Return high[i] - low[i] for each dimension.");

// //         .def("setBounds",
// //              py::overload_cast<const RealVectorBounds &>(
// //                  &RealVectorStateSpace::setBounds),
// //              py::arg("bounds"),
// //              "Set the same bounds for all dimensions.")

// //         .def("getBounds",
// //              &RealVectorStateSpace::getBounds,
// //              "Return the current bounds.")

// //         .def("getMaximumExtent",
// //              &RealVectorStateSpace::getMaximumExtent,
// //              "Return the maximum distance between any two states.")

// //         .def("allocState",
// //              [](RealVectorStateSpace &space) {
// //                  // allocState returns a raw State*; we cast to StateType
// //                  // and return it with a keep-alive so the space outlives it.
// //                  return space.allocState()
// //                               ->as<RealVectorStateSpace::StateType>();
// //              },
// //              py::return_value_policy::reference_internal,
// //              "Allocate a new state (owned by the space).")

// //         .def("freeState",
// //              [](RealVectorStateSpace &space,
// //                 RealVectorStateSpace::StateType *st) {
// //                  space.freeState(st);
// //              }, py::arg("state"),
// //              "Free a previously allocated state.")

// //         .def("distance",
// //              [](RealVectorStateSpace &space,
// //                 const RealVectorStateSpace::StateType *s1,
// //                 const RealVectorStateSpace::StateType *s2) {
// //                  return space.distance(s1, s2);
// //              }, py::arg("s1"), py::arg("s2"),
// //              "Return the Euclidean distance between two states.")

// //         .def("interpolate",
// //              [](RealVectorStateSpace &space,
// //                 const RealVectorStateSpace::StateType *from,
// //                 const RealVectorStateSpace::StateType *to,
// //                 double t,
// //                 RealVectorStateSpace::StateType *result) {
// //                  space.interpolate(from, to, t, result);
// //              }, py::arg("from"), py::arg("to"),
// //              py::arg("t"), py::arg("result"),
// //              "Interpolate between two states at fraction t in [0,1].")

// //         .def("satisfiesBounds",
// //              [](RealVectorStateSpace &space,
// //                 const RealVectorStateSpace::StateType *s) {
// //                  return space.satisfiesBounds(s);
// //              }, py::arg("state"),
// //              "Return True if the state is within bounds.")

// //         .def("enforceBounds",
// //              [](RealVectorStateSpace &space,
// //                 RealVectorStateSpace::StateType *s) {
// //                  space.enforceBounds(s);
// //              }, py::arg("state"),
// //              "Clamp the state to lie within bounds.")

// //         .def("getName",    &RealVectorStateSpace::getName,
// //              "Return the name of the space.")
// //         .def("setName",    &RealVectorStateSpace::setName,  py::arg("name"),
// //              "Set the name of the space.")
// //         .def("__repr__",
// //              [](const RealVectorStateSpace &s) {
// //                  return "<RealVectorStateSpace dim=" +
// //                         std::to_string(s.getDimension()) + ">";
// //              });
// // }


















// #pragma once
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>

// #include <ompl/base/State.h>
// #include <ompl/base/StateSpace.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/spaces/RealVectorBounds.h>

// namespace py = pybind11;
// using namespace ompl::base;

// // ---------------------------------------------------------------------------
// // Helper free functions — MSVC handles these more reliably than inline lambdas
// // inside heavily templated pybind11 .def() chains
// // ---------------------------------------------------------------------------

// static unsigned int rvbounds_get_dimension(const RealVectorBounds &b)
// {
//     return static_cast<unsigned int>(b.low.size());
// }

// static double rvbounds_get_volume(const RealVectorBounds &b)
// {
//     return b.getVolume();
// }

// static std::vector<double> rvbounds_get_difference(const RealVectorBounds &b)
// {
//     return b.getDifference();
// }

// static std::vector<double> rvbounds_get_low(const RealVectorBounds &b)
// {
//     return b.low;
// }

// static std::vector<double> rvbounds_get_high(const RealVectorBounds &b)
// {
//     return b.high;
// }

// // -- RealVectorStateSpace::StateType helpers --------------------------------

// static double rvstate_getitem(const RealVectorStateSpace::StateType &s, unsigned int i)
// {
//     return s[i];
// }

// static void rvstate_setitem(RealVectorStateSpace::StateType &s, unsigned int i, double v)
// {
//     s[i] = v;
// }

// // -- RealVectorStateSpace helpers ------------------------------------------

// static RealVectorStateSpace::StateType *rvspace_alloc(RealVectorStateSpace &space)
// {
//     return space.allocState()->as<RealVectorStateSpace::StateType>();
// }

// static void rvspace_free(RealVectorStateSpace &space, RealVectorStateSpace::StateType *st)
// {
//     space.freeState(st);
// }

// static double rvspace_distance(RealVectorStateSpace &space,
//                                const RealVectorStateSpace::StateType *s1,
//                                const RealVectorStateSpace::StateType *s2)
// {
//     return space.distance(s1, s2);
// }

// static void rvspace_interpolate(RealVectorStateSpace &space,
//                                 const RealVectorStateSpace::StateType *from,
//                                 const RealVectorStateSpace::StateType *to,
//                                 double t,
//                                 RealVectorStateSpace::StateType *result)
// {
//     space.interpolate(from, to, t, result);
// }

// static bool rvspace_satisfies_bounds(RealVectorStateSpace &space,
//                                      const RealVectorStateSpace::StateType *s)
// {
//     return space.satisfiesBounds(s);
// }

// static void rvspace_enforce_bounds(RealVectorStateSpace &space,
//                                    RealVectorStateSpace::StateType *s)
// {
//     space.enforceBounds(s);
// }

// static std::string rvspace_repr(const RealVectorStateSpace &s)
// {
//     return "<RealVectorStateSpace dim=" + std::to_string(s.getDimension()) + ">";
// }

// // ---------------------------------------------------------------------------
// // Binding function
// // ---------------------------------------------------------------------------

// inline void bind_base_state_space(py::module_ &m)
// {
//     // ------------------------------------------------------------------ //
//     //  RealVectorBounds
//     // ------------------------------------------------------------------ //
//     py::class_<RealVectorBounds>(m, "RealVectorBounds",
//         "Axis-aligned bounding box for a RealVectorStateSpace.\n\n"
//         "C++ header: ompl/base/spaces/RealVectorBounds.h\n"
//         "C++ class:  ompl::base::RealVectorBounds")

//         .def(py::init<unsigned int>(), py::arg("dim"),
//              "Create bounds for the given number of dimensions.\n"
//              "C++: RealVectorBounds bounds(dim);")

//         .def("setLow",
//              py::overload_cast<double>(&RealVectorBounds::setLow),
//              py::arg("value"),
//              "Set ALL lower bounds to the same value.\n"
//              "C++: bounds.setLow(value);")

//         .def("setLow",
//              py::overload_cast<unsigned int, double>(&RealVectorBounds::setLow),
//              py::arg("index"), py::arg("value"),
//              "Set the lower bound for one specific dimension.\n"
//              "C++: bounds.setLow(index, value);")

//         .def("setHigh",
//              py::overload_cast<double>(&RealVectorBounds::setHigh),
//              py::arg("value"),
//              "Set ALL upper bounds to the same value.\n"
//              "C++: bounds.setHigh(value);")

//         .def("setHigh",
//              py::overload_cast<unsigned int, double>(&RealVectorBounds::setHigh),
//              py::arg("index"), py::arg("value"),
//              "Set the upper bound for one specific dimension.\n"
//              "C++: bounds.setHigh(index, value);")

//         .def("getLow",  &rvbounds_get_low,
//              "Return the list of lower bounds.\n"
//              "C++: bounds.low  (public std::vector<double>)")

//         .def("getHigh", &rvbounds_get_high,
//              "Return the list of upper bounds.\n"
//              "C++: bounds.high  (public std::vector<double>)")

//         .def("check", &RealVectorBounds::check,
//              "Validate bounds — throws if low[i] >= high[i].\n"
//              "C++: bounds.check();")

//         .def("getDimension", &rvbounds_get_dimension,
//              "Return the number of dimensions (size of low/high vectors).\n"
//              "C++: bounds.low.size()")

//         .def("getVolume", &rvbounds_get_volume,
//              "Return the volume of the bounded region = product of (high[i]-low[i]).\n"
//              "C++: bounds.getVolume()")

//         .def("getDifference", &rvbounds_get_difference,
//              "Return high[i] - low[i] for each dimension.\n"
//              "C++: bounds.getDifference()");

//     // ------------------------------------------------------------------ //
//     //  RealVectorStateSpace::StateType
//     // ------------------------------------------------------------------ //
//     py::class_<RealVectorStateSpace::StateType>(m, "RealVectorState",
//         "A single state in a RealVectorStateSpace — an array of doubles.\n\n"
//         "C++ type: ompl::base::RealVectorStateSpace::StateType\n"
//         "Do NOT construct directly. Use space.allocState() instead.")

//         .def("__getitem__", &rvstate_getitem, py::arg("index"),
//              "Read value at dimension index.\n"
//              "C++: state->values[index]  or  (*state)[index]")

//         .def("__setitem__", &rvstate_setitem, py::arg("index"), py::arg("value"),
//              "Write value at dimension index.\n"
//              "C++: state->values[index] = value");

//     // ------------------------------------------------------------------ //
//     //  RealVectorStateSpace
//     // ------------------------------------------------------------------ //
//     py::class_<RealVectorStateSpace, StateSpace,
//                std::shared_ptr<RealVectorStateSpace>>(
//         m, "RealVectorStateSpace",
//         "A state space where each state is a point in R^n.\n\n"
//         "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
//         "C++ class:  ompl::base::RealVectorStateSpace\n"
//         "Typical use: robot with n joints, or a point in n-dimensional space.")

//         .def(py::init<unsigned int>(), py::arg("dim") = 0,
//              "Construct with the given number of dimensions.\n"
//              "C++: auto space = std::make_shared<RealVectorStateSpace>(dim);")

//         .def("getDimension",
//              &RealVectorStateSpace::getDimension,
//              "Return the number of dimensions.\n"
//              "C++: space->getDimension()")

//         .def("setBounds",
//              py::overload_cast<const RealVectorBounds &>(
//                  &RealVectorStateSpace::setBounds),
//              py::arg("bounds"),
//              "Set the bounds for all dimensions.\n"
//              "C++: space->setBounds(bounds);")

//         .def("getBounds",
//              &RealVectorStateSpace::getBounds,
//              "Return the current bounds.\n"
//              "C++: space->getBounds()")

//         .def("getMaximumExtent",
//              &RealVectorStateSpace::getMaximumExtent,
//              "Return the maximum possible distance between any two states.\n"
//              "For R^n with equal bounds this is the diagonal length.\n"
//              "C++: space->getMaximumExtent()")

//         .def("allocState",   &rvspace_alloc,
//              py::return_value_policy::reference_internal,
//              "Allocate and return a new state (memory managed by space).\n"
//              "Always pair with freeState() when done.\n"
//              "C++: space->allocState()->as<RealVectorStateSpace::StateType>()")

//         .def("freeState",    &rvspace_free,    py::arg("state"),
//              "Free a previously allocated state.\n"
//              "C++: space->freeState(state);")

//         .def("distance",     &rvspace_distance, py::arg("s1"), py::arg("s2"),
//              "Return the Euclidean distance between two states.\n"
//              "C++: space->distance(s1, s2)")

//         .def("interpolate",  &rvspace_interpolate,
//              py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
//              "Interpolate between two states at fraction t in [0,1].\n"
//              "t=0 -> from, t=1 -> to, t=0.5 -> midpoint.\n"
//              "C++: space->interpolate(from, to, t, result);")

//         .def("satisfiesBounds", &rvspace_satisfies_bounds, py::arg("state"),
//              "Return True if the state is within the set bounds.\n"
//              "C++: space->satisfiesBounds(state)")

//         .def("enforceBounds",   &rvspace_enforce_bounds,   py::arg("state"),
//              "Clamp the state values to lie within bounds (modifies in place).\n"
//              "C++: space->enforceBounds(state);")

//         .def("getName",    &RealVectorStateSpace::getName,
//              "Return the name of this space.\n"
//              "C++: space->getName()")

//         .def("setName",    &RealVectorStateSpace::setName,  py::arg("name"),
//              "Set a name for this space (useful for debugging).\n"
//              "C++: space->setName(name);")

//         .def("__repr__",   &rvspace_repr);
// }



#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>

namespace py = pybind11;
using namespace ompl::base;

// ---------------------------------------------------------------------------
// Helper free functions — MSVC handles these more reliably than inline lambdas
// ---------------------------------------------------------------------------

static unsigned int rvbounds_get_dimension(const RealVectorBounds &b)
{
    return static_cast<unsigned int>(b.low.size());
}

static std::vector<double> rvbounds_get_low(const RealVectorBounds &b)
{
    return b.low;
}

static std::vector<double> rvbounds_get_high(const RealVectorBounds &b)
{
    return b.high;
}

static double rvstate_getitem(const RealVectorStateSpace::StateType &s, unsigned int i)
{
    return s[i];
}

static void rvstate_setitem(RealVectorStateSpace::StateType &s, unsigned int i, double v)
{
    s[i] = v;
}

static RealVectorStateSpace::StateType *rvspace_alloc(RealVectorStateSpace &space)
{
    return space.allocState()->as<RealVectorStateSpace::StateType>();
}

static void rvspace_free(RealVectorStateSpace &space, RealVectorStateSpace::StateType *st)
{
    space.freeState(st);
}

static double rvspace_distance(RealVectorStateSpace &space,
                               const RealVectorStateSpace::StateType *s1,
                               const RealVectorStateSpace::StateType *s2)
{
    return space.distance(s1, s2);
}

static void rvspace_interpolate(RealVectorStateSpace &space,
                                const RealVectorStateSpace::StateType *from,
                                const RealVectorStateSpace::StateType *to,
                                double t,
                                RealVectorStateSpace::StateType *result)
{
    space.interpolate(from, to, t, result);
}

static bool rvspace_satisfies_bounds(RealVectorStateSpace &space,
                                     const RealVectorStateSpace::StateType *s)
{
    return space.satisfiesBounds(s);
}

static void rvspace_enforce_bounds(RealVectorStateSpace &space,
                                   RealVectorStateSpace::StateType *s)
{
    space.enforceBounds(s);
}

static std::string rvspace_repr(const RealVectorStateSpace &s)
{
    return "<RealVectorStateSpace dim=" + std::to_string(s.getDimension()) + ">";
}

// ---------------------------------------------------------------------------
// Binding function
// ---------------------------------------------------------------------------

inline void bind_base_state_space(py::module_ &m)
{
    // ------------------------------------------------------------------ //
    //  StateSpace (abstract base) — MUST be registered first so that
    //  RealVectorStateSpace can declare it as a base class in pybind11.
    //  Without this, pybind11 throws "referenced unknown base type".
    // ------------------------------------------------------------------ //
    py::class_<StateSpace, std::shared_ptr<StateSpace>>(m, "StateSpace",
        "Abstract base class for all OMPL state spaces.\n\n"
        "C++ header: ompl/base/StateSpace.h\n"
        "C++ class:  ompl::base::StateSpace\n"
        "You do not instantiate this directly — use a concrete subclass\n"
        "like RealVectorStateSpace, SE2StateSpace, etc.")

        .def("getDimension",     &StateSpace::getDimension,
             "Return the number of dimensions in this space.\n"
             "C++: space->getDimension()")

        .def("getMaximumExtent", &StateSpace::getMaximumExtent,
             "Return the maximum distance between any two states.\n"
             "C++: space->getMaximumExtent()")

        .def("getName",          &StateSpace::getName,
             "Return the name of this space.\n"
             "C++: space->getName()")

        .def("setName",          &StateSpace::setName, py::arg("name"),
             "Set a name for this space.\n"
             "C++: space->setName(name);")

        .def("satisfiesBounds",  &StateSpace::satisfiesBounds, py::arg("state"),
             "Return True if the given state satisfies the space bounds.\n"
             "C++: space->satisfiesBounds(state)")

        .def("enforceBounds",    &StateSpace::enforceBounds, py::arg("state"),
             "Clamp the state to lie within bounds.\n"
             "C++: space->enforceBounds(state);")

        .def("distance",         &StateSpace::distance,
             py::arg("s1"), py::arg("s2"),
             "Return the distance between two states.\n"
             "C++: space->distance(s1, s2)");

    // ------------------------------------------------------------------ //
    //  RealVectorBounds
    // ------------------------------------------------------------------ //
    py::class_<RealVectorBounds>(m, "RealVectorBounds",
        "Axis-aligned bounding box for a RealVectorStateSpace.\n\n"
        "C++ header: ompl/base/spaces/RealVectorBounds.h\n"
        "C++ class:  ompl::base::RealVectorBounds")

        .def(py::init<unsigned int>(), py::arg("dim"),
             "Create bounds for the given number of dimensions.\n"
             "C++: RealVectorBounds bounds(dim);")

        .def("setLow",
             py::overload_cast<double>(&RealVectorBounds::setLow),
             py::arg("value"),
             "Set ALL lower bounds to the same value.\n"
             "C++: bounds.setLow(value);")

        .def("setLow",
             py::overload_cast<unsigned int, double>(&RealVectorBounds::setLow),
             py::arg("index"), py::arg("value"),
             "Set the lower bound for one specific dimension.\n"
             "C++: bounds.setLow(index, value);")

        .def("setHigh",
             py::overload_cast<double>(&RealVectorBounds::setHigh),
             py::arg("value"),
             "Set ALL upper bounds to the same value.\n"
             "C++: bounds.setHigh(value);")

        .def("setHigh",
             py::overload_cast<unsigned int, double>(&RealVectorBounds::setHigh),
             py::arg("index"), py::arg("value"),
             "Set the upper bound for one specific dimension.\n"
             "C++: bounds.setHigh(index, value);")

        .def("getLow",        &rvbounds_get_low,
             "Return the list of lower bounds.\n"
             "C++: bounds.low  (public std::vector<double>)")

        .def("getHigh",       &rvbounds_get_high,
             "Return the list of upper bounds.\n"
             "C++: bounds.high  (public std::vector<double>)")

        .def("check",         &RealVectorBounds::check,
             "Validate bounds — throws if low[i] >= high[i].\n"
             "C++: bounds.check();")

        .def("getDimension",  &rvbounds_get_dimension,
             "Return the number of dimensions (size of low/high vectors).\n"
             "C++: bounds.low.size()")

        .def("getVolume",     &RealVectorBounds::getVolume,
             "Return the volume of the bounded region.\n"
             "C++: bounds.getVolume()")

        .def("getDifference", &RealVectorBounds::getDifference,
             "Return high[i] - low[i] for each dimension.\n"
             "C++: bounds.getDifference()");

    // ------------------------------------------------------------------ //
    //  RealVectorStateSpace::StateType
    // ------------------------------------------------------------------ //
    py::class_<RealVectorStateSpace::StateType>(m, "RealVectorState",
        "A single state in a RealVectorStateSpace — an array of doubles.\n\n"
        "C++ type: ompl::base::RealVectorStateSpace::StateType\n"
        "Do NOT construct directly. Use space.allocState() instead.")

        .def("__getitem__", &rvstate_getitem, py::arg("index"),
             "Read value at dimension index.\n"
             "C++: state->values[index]  or  (*state)[index]")

        .def("__setitem__", &rvstate_setitem, py::arg("index"), py::arg("value"),
             "Write value at dimension index.\n"
             "C++: state->values[index] = value");

    // ------------------------------------------------------------------ //
    //  RealVectorStateSpace
    // ------------------------------------------------------------------ //
    py::class_<RealVectorStateSpace, StateSpace,
               std::shared_ptr<RealVectorStateSpace>>(
        m, "RealVectorStateSpace",
        "A state space where each state is a point in R^n.\n\n"
        "C++ header: ompl/base/spaces/RealVectorStateSpace.h\n"
        "C++ class:  ompl::base::RealVectorStateSpace\n"
        "Typical use: robot with n joints, or a point in n-dimensional space.")

        .def(py::init<unsigned int>(), py::arg("dim") = 0,
             "Construct with the given number of dimensions.\n"
             "C++: auto space = std::make_shared<RealVectorStateSpace>(dim);")

        .def("getDimension",     &RealVectorStateSpace::getDimension,
             "Return the number of dimensions.\n"
             "C++: space->getDimension()")

        .def("setBounds",
             py::overload_cast<const RealVectorBounds &>(
                 &RealVectorStateSpace::setBounds),
             py::arg("bounds"),
             "Set the bounds for all dimensions.\n"
             "C++: space->setBounds(bounds);")

        .def("getBounds",        &RealVectorStateSpace::getBounds,
             "Return the current bounds.\n"
             "C++: space->getBounds()")

        .def("getMaximumExtent", &RealVectorStateSpace::getMaximumExtent,
             "Return the maximum possible distance between any two states.\n"
             "C++: space->getMaximumExtent()")

        .def("allocState",       &rvspace_alloc,
             py::return_value_policy::reference_internal,
             "Allocate and return a new state (memory managed by the space).\n"
             "Always pair with freeState() when done.\n"
             "C++: space->allocState()->as<RealVectorStateSpace::StateType>()")

        .def("freeState",        &rvspace_free,        py::arg("state"),
             "Free a previously allocated state.\n"
             "C++: space->freeState(state);")

        .def("distance",         &rvspace_distance,    py::arg("s1"), py::arg("s2"),
             "Return the Euclidean distance between two states.\n"
             "C++: space->distance(s1, s2)")

        .def("interpolate",      &rvspace_interpolate,
             py::arg("from"), py::arg("to"), py::arg("t"), py::arg("result"),
             "Interpolate between two states at fraction t in [0,1].\n"
             "t=0 -> from,  t=1 -> to,  t=0.5 -> midpoint.\n"
             "C++: space->interpolate(from, to, t, result);")

        .def("satisfiesBounds",  &rvspace_satisfies_bounds, py::arg("state"),
             "Return True if the state is within the set bounds.\n"
             "C++: space->satisfiesBounds(state)")

        .def("enforceBounds",    &rvspace_enforce_bounds,   py::arg("state"),
             "Clamp the state values to lie within bounds (modifies in place).\n"
             "C++: space->enforceBounds(state);")

        .def("getName",          &RealVectorStateSpace::getName,
             "Return the name of this space.\n"
             "C++: space->getName()")

        .def("setName",          &RealVectorStateSpace::setName, py::arg("name"),
             "Set a name for this space (useful for debugging).\n"
             "C++: space->setName(name);")

        .def("__repr__",         &rvspace_repr);
}