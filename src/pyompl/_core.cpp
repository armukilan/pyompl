// #include <pybind11/pybind11.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>

// namespace py = pybind11;
// namespace ob = ompl::base;

// PYBIND11_MODULE(_core, m) {
//     m.doc() = "pyompl core bindings";

//     m.def("hello", []() {
//         return "OMPL successfully initialized";
//     });

//     py::class_<ob::RealVectorStateSpace>(m, "RealVectorStateSpace")
//         .def(py::init<unsigned int>(), py::arg("dim") = 1)
//         .def("getDimension", &ob::RealVectorStateSpace::getDimension);
// }

#include <pybind11/pybind11.h>
namespace py = pybind11;

// Include each binding header here as we add them
#include "bindings/base_state_space.h"

PYBIND11_MODULE(_core, m) {
    m.doc() = "pyompl — Python bindings for OMPL 2.0.0";

    bind_base_state_space(m);
    // bind_se2_state_space(m);   // Phase 1 Step 2
    // bind_space_information(m); // Phase 1 Step 3
    // bind_problem_definition(m);// Phase 1 Step 4
    // bind_planners(m);          // Phase 2
}