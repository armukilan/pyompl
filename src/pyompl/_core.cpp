#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "bindings/state_space.h"       // StateSpace + CompoundStateSpace — MUST be first
#include "bindings/base_state_space.h"  // RealVectorBounds + RealVectorStateSpace

PYBIND11_MODULE(_core, m) {
    m.doc() = "pyompl — Python bindings for OMPL 2.0.0";

    bind_state_space(m);        // registers StateSpace + CompoundStateSpace
    bind_base_state_space(m);   // registers RealVectorBounds + RealVectorStateSpace
}