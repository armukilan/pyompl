#include <pybind11/pybind11.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace py = pybind11;
namespace ob = ompl::base;

PYBIND11_MODULE(_core, m) {
    m.doc() = "pyompl core bindings";

    m.def("hello", []() {
        return "OMPL successfully initialized";
    });

    py::class_<ob::RealVectorStateSpace>(m, "RealVectorStateSpace")
        .def(py::init<unsigned int>(), py::arg("dim") = 1)
        .def("getDimension", &ob::RealVectorStateSpace::getDimension);
}