#pragma once

#include <pybind11/pybind11.h>
#include "ompl/base/State.h"

namespace py = pybind11;

inline void bind_state(py::module &m)
{
    using namespace ompl::base;

    // State: abstract base, protected ctor/dtor in C++.
    // No py::init() exposed — states are only ever created via StateSpace::allocState().
    // py::nodelete — Python must never own/free a State; the StateSpace owns it.
    py::class_<State, std::unique_ptr<State, py::nodelete>>(m, "State")
        .def("__repr__", [](const State &) {
            return "<ompl.base.State>";
        });

    // CompoundState : State
    // Public ctor/dtor in C++, but still nodelete here — ownership stays with
    // whatever StateSpace allocates it once StateSpace.h bindings exist.
    py::class_<CompoundState, State, std::unique_ptr<CompoundState, py::nodelete>>(m, "CompoundState")
        .def(py::init<>())
        .def("__getitem__", [](CompoundState &s, unsigned int i) -> State * {
            return s.components[i];
        }, py::return_value_policy::reference_internal)
        .def("__repr__", [](const CompoundState &) {
            return "<ompl.base.CompoundState>";
        });
}