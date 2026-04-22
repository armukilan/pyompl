#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "bindings/state_space.h"
#include "bindings/base_state_space.h"
#include "bindings/so2_state_space.h"
#include "bindings/so3_state_space.h"
#include "bindings/se2_state_space.h"

PYBIND11_MODULE(_core, m) {
    m.doc() = "pyompl - Python bindings for OMPL 2.0.0";
    bind_state_space(m);
    bind_base_state_space(m);
    bind_so2_state_space(m);
    bind_so3_state_space(m);
    bind_se2_state_space(m);
}
