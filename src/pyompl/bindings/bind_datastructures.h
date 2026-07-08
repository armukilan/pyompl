#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Undirected graph with weighted edges + Dijkstra
#include <ompl/datastructures/AdjacencyList.h>

// Index permutation (derives std::vector<int>)
#include <ompl/datastructures/Permutation.h>

// Dynamic single-source-shortest-path structure
#include <ompl/datastructures/DynamicSSSP.h>

#include <utility>
#include <vector>
#include <list>
#include <cstddef>

namespace py = pybind11;

// =============================================================================
// WHAT IS OMPL DATASTRUCTURES?
// -----------------------------
// The datastructures/ folder contains generic containers/algorithms used
// throughout OMPL's planners:
//   AdjacencyList — undirected weighted graph + Dijkstra shortest path
//   Permutation   — shuffled index array (vector<int> subclass)
//   DynamicSSSP   — incrementally-updated single-source-shortest-path graph
//
// SCOPE OF THIS FILE:
// Only these three CONCRETE (non-template) classes are bound here.
// NearestNeighbors*, Grid*, BinaryHeap, PDF, GreedyKCenters, LPAstarOnGraph
// are C++ templates with no fixed T yet — they are deferred until `base`
// gives us a concrete type (e.g. ompl::base::State*) to instantiate against.
// Only PUBLIC members are bound; private/protected members are intentionally
// left out (see inline notes below each class).
// =============================================================================

// ===========================================================================
// AdjacencyList helpers
// (out-param C++ signatures are converted to return values for Python)
// ===========================================================================

static std::vector<int> adjlist_get_neighbors(const ompl::AdjacencyList &self, int vtx)
{
    // C++: self.getNeighbors(vtx, std::vector<int>&)
    // Returns the plain list of neighbor vertex ids.
    std::vector<int> nbrs;
    self.getNeighbors(vtx, nbrs);
    return nbrs;
}

static std::vector<std::pair<int, double>> adjlist_get_neighbors_weighted(
    const ompl::AdjacencyList &self, int vtx)
{
    // C++: self.getNeighbors(vtx, std::vector<std::pair<int,double>>&)
    // Returns (neighbor id, edge weight) pairs.
    std::vector<std::pair<int, double>> nbrs;
    self.getNeighbors(vtx, nbrs);
    return nbrs;
}

static std::pair<bool, std::vector<int>> adjlist_dijkstra(
    const ompl::AdjacencyList &self, int v1, int v2)
{
    // C++: self.dijkstra(v1, v2, std::vector<int>& path)
    // Returns (found, path).
    std::vector<int> path;
    bool found = self.dijkstra(v1, v2, path);
    return std::make_pair(found, path);
}

static std::pair<std::vector<int>, std::vector<double>> adjlist_dijkstra_all(
    const ompl::AdjacencyList &self, int vtx)
{
    // C++: self.dijkstra(vtx, predecessors, distance)
    // Returns (predecessors, distance) for every vertex.
    std::vector<int> predecessors;
    std::vector<double> distance;
    self.dijkstra(vtx, predecessors, distance);
    return std::make_pair(predecessors, distance);
}

// ===========================================================================
// DynamicSSSP helpers
// (affectedVertices out-param converted to a return value)
// ===========================================================================

static std::list<std::size_t> sssp_add_edge(
    ompl::DynamicSSSP &self, std::size_t v, std::size_t w, double weight,
    bool collectVertices)
{
    // C++: self.addEdge(v, w, weight, collectVertices, affectedVertices&)
    // Returns the affected-vertices list (empty if collectVertices == False).
    std::list<std::size_t> affected;
    self.addEdge(v, w, weight, collectVertices, affected);
    return affected;
}

static std::list<std::size_t> sssp_remove_edge(
    ompl::DynamicSSSP &self, std::size_t v, std::size_t w, bool collectVertices)
{
    // C++: self.removeEdge(v, w, collectVertices, affectedVertices&)
    // Returns the affected-vertices list (empty if collectVertices == False).
    std::list<std::size_t> affected;
    self.removeEdge(v, w, collectVertices, affected);
    return affected;
}

// ===========================================================================
// Binding function
// ===========================================================================

inline void bind_datastructures(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // Create a submodule for datastructures
    // -----------------------------------------------------------------------
    auto ds = m.def_submodule("datastructures",
        "OMPL generic data structures: graphs, permutations, dynamic SSSP.\n\n"
        "Usage:\n"
        "  import pyompl\n"
        "  g = pyompl.datastructures.AdjacencyList(5)\n"
        "  perm = pyompl.datastructures.Permutation(10)\n"
        "  sssp = pyompl.datastructures.DynamicSSSP()");

    // =======================================================================
    // AdjacencyList
    // C++ class:  ompl::AdjacencyList
    // C++ header: ompl/datastructures/AdjacencyList.h
    // =======================================================================
    py::class_<ompl::AdjacencyList>(ds, "AdjacencyList",
        "An undirected, weighted graph with connected-component queries\n"
        "and Dijkstra shortest-path search.\n\n"
        "C++ class:  ompl::AdjacencyList\n"
        "C++ header: ompl/datastructures/AdjacencyList.h\n\n"
        "NOTE: vertex removal is not supported; only edge add/remove.\n"
        "NOTE: the protected mutex `lock_` is not exposed to Python.")

        .def(py::init<>(),
            "Construct an empty graph (0 vertices).\n"
            "C++: ompl::AdjacencyList list;")

        .def(py::init<int>(), py::arg("n"),
            "Construct a graph with n vertices and no edges.\n"
            "C++: ompl::AdjacencyList list(n);")

        .def("clear", &ompl::AdjacencyList::clear,
            "Remove all vertices and edges.\n"
            "C++: list.clear()")

        .def("addVertex", &ompl::AdjacencyList::addVertex,
            "Add a new vertex; returns its id.\n"
            "C++: list.addVertex()")

        .def("numVertices", &ompl::AdjacencyList::numVertices,
            "Return the number of vertices.\n"
            "C++: list.numVertices()")

        .def("vertexExists", &ompl::AdjacencyList::vertexExists, py::arg("v"),
            "Return True if vertex v exists.\n"
            "C++: list.vertexExists(v)")

        .def("inSameComponent", &ompl::AdjacencyList::inSameComponent,
            py::arg("v1"), py::arg("v2"),
            "Return True if v1 and v2 are in the same connected component.\n\n"
            "C++: list.inSameComponent(v1, v2)\n\n"
            "NOTE: not guaranteed correct if edges have ever been removed.")

        .def("numConnectedComponents", &ompl::AdjacencyList::numConnectedComponents,
            "Return the number of connected components.\n\n"
            "C++: list.numConnectedComponents()\n\n"
            "NOTE: not guaranteed correct if edges have ever been removed.")

        .def("getComponentID", &ompl::AdjacencyList::getComponentID, py::arg("vtx"),
            "Return the connected-component id that vtx belongs to.\n"
            "C++: list.getComponentID(vtx)")

        .def("addEdge", &ompl::AdjacencyList::addEdge,
            py::arg("v1"), py::arg("v2"), py::arg("weight") = 1.0,
            "Add an edge between v1 and v2 with an optional weight.\n\n"
            "C++: list.addEdge(v1, v2, weight)\n\n"
            "Returns False if the edge already exists.")

        .def("removeEdge", &ompl::AdjacencyList::removeEdge, py::arg("v1"), py::arg("v2"),
            "Remove the edge between v1 and v2.\n\n"
            "C++: list.removeEdge(v1, v2)\n\n"
            "NOTE: using this trashes connected-component tracking.")

        .def("numEdges", &ompl::AdjacencyList::numEdges,
            "Return the number of edges.\n"
            "C++: list.numEdges()")

        .def("getEdgeWeight", &ompl::AdjacencyList::getEdgeWeight, py::arg("v1"), py::arg("v2"),
            "Return the weight of the edge between v1 and v2.\n\n"
            "C++: list.getEdgeWeight(v1, v2)\n\n"
            "Throws if the edge does not exist.")

        .def("setEdgeWeight", &ompl::AdjacencyList::setEdgeWeight,
            py::arg("v1"), py::arg("v2"), py::arg("weight"),
            "Update the weight of an existing edge.\n"
            "C++: list.setEdgeWeight(v1, v2, weight)")

        .def("edgeExists", &ompl::AdjacencyList::edgeExists, py::arg("v1"), py::arg("v2"),
            "Return True if an edge exists between v1 and v2.\n"
            "C++: list.edgeExists(v1, v2)")

        .def("numNeighbors", &ompl::AdjacencyList::numNeighbors, py::arg("vtx"),
            "Return the number of adjacent vertices for vtx.\n"
            "C++: list.numNeighbors(vtx)")

        .def("getNeighbors", &adjlist_get_neighbors, py::arg("vtx"),
            "Return the list of neighbor vertex ids for vtx.\n\n"
            "C++: list.getNeighbors(vtx, std::vector<int>&)\n"
            "(out-param converted to a return value)")

        .def("getNeighborsWeighted", &adjlist_get_neighbors_weighted, py::arg("vtx"),
            "Return (neighbor_id, edge_weight) pairs for vtx.\n\n"
            "C++: list.getNeighbors(vtx, std::vector<std::pair<int,double>>&)\n"
            "(out-param converted to a return value)")

        .def("dijkstra", &adjlist_dijkstra, py::arg("v1"), py::arg("v2"),
            "Dijkstra shortest path search from v1 to v2.\n\n"
            "C++: list.dijkstra(v1, v2, std::vector<int>& path)\n\n"
            "Returns (found: bool, path: list[int]).\n"
            "(out-param converted to a return value)")

        .def("dijkstraAll", &adjlist_dijkstra_all, py::arg("vtx"),
            "Dijkstra shortest path search from vtx to every other vertex.\n\n"
            "C++: list.dijkstra(vtx, predecessors, distance)\n\n"
            "Returns (predecessors: list[int], distance: list[float]).\n"
            "A predecessor equal to the vertex itself means unreachable.\n"
            "(out-param converted to a return value)");
        // NOTE: protected member `lock_` intentionally NOT bound.

    // =======================================================================
    // Permutation
    // C++ class:  ompl::Permutation
    // C++ header: ompl/datastructures/Permutation.h
    // =======================================================================
    py::class_<ompl::Permutation>(ds, "Permutation",
        "A shuffled permutation of indices 0..n-1.\n\n"
        "C++ class:  ompl::Permutation (derives std::vector<int>)\n"
        "C++ header: ompl/datastructures/Permutation.h\n\n"
        "WHY USE IT?\n"
        "Faster than repeated std::random_shuffle calls since the RNG\n"
        "is allocated once. Used internally where planners need a\n"
        "randomized visiting order.\n\n"
        "NOTE: pybind11 does not auto-inherit std::vector<int> bindings,\n"
        "so container access (len/index/iterate) is exposed manually below.\n"
        "NOTE: the private RNG member `generator_` is not exposed to Python.")

        .def(py::init<std::size_t>(), py::arg("n"),
            "Create a permutation of the numbers 0, ..., n - 1.\n"
            "C++: ompl::Permutation perm(n);")

        .def("permute", &ompl::Permutation::permute, py::arg("n"),
            "Re-shuffle to a fresh permutation of 0, ..., n - 1.\n"
            "C++: perm.permute(n)")

        .def("__len__", [](const ompl::Permutation &self)
             { return self.size(); },
            "Number of elements (inherited from std::vector<int>::size()).")

        .def("__getitem__", [](const ompl::Permutation &self, std::size_t i)
             {
                 if (i >= self.size())
                     throw py::index_error();
                 return self[i];
             },
            "Index access (inherited from std::vector<int>::operator[]).")

        .def("__setitem__", [](ompl::Permutation &self, std::size_t i, int value)
             {
                 if (i >= self.size())
                     throw py::index_error();
                 self[i] = value;
             },
            "Index assignment (inherited from std::vector<int>::operator[]).")

        .def("__iter__", [](const ompl::Permutation &self)
             { return py::make_iterator(self.begin(), self.end()); },
             py::keep_alive<0, 1>(),
            "Iterate over the permuted indices.");

    // =======================================================================
    // DynamicSSSP
    // C++ class:  ompl::DynamicSSSP
    // C++ header: ompl/datastructures/DynamicSSSP.h
    // =======================================================================
    py::class_<ompl::DynamicSSSP>(ds, "DynamicSSSP",
        "Incrementally-updated single-source-shortest-path structure.\n\n"
        "C++ class:  ompl::DynamicSSSP\n"
        "C++ header: ompl/datastructures/DynamicSSSP.h\n\n"
        "Implementation based on:\n"
        "  Ramalingam & Reps, 'On the computational complexity of dynamic\n"
        "  graph problems.' Theor. Comput. Sci. 158(1-2), 1996.\n\n"
        "NOTE: assumes no two paths share the same cost (valid when edge\n"
        "weights have some randomness, as in sampling-based planners).\n"
        "NOTE: private members `graph_`, `distance_`, `parent_` and the\n"
        "private nested comparator class `IsLessThan` are not exposed.")

        .def(py::init<>(),
            "Construct an empty dynamic SSSP structure.\n"
            "C++: ompl::DynamicSSSP sssp;")

        .def("clear", &ompl::DynamicSSSP::clear,
            "Remove all vertices, edges, and distance/parent info.\n"
            "C++: sssp.clear()")

        .def("addVertex", &ompl::DynamicSSSP::addVertex, py::arg("id"),
            "Add a vertex with the given id.\n\n"
            "C++: sssp.addVertex(id)\n\n"
            "Vertex 0 is treated as the source (distance 0); all\n"
            "others start at infinity until connected.")

        .def("addEdge", &sssp_add_edge,
            py::arg("v"), py::arg("w"), py::arg("weight"),
            py::arg("collect_vertices") = false,
            "Add an edge (v, w) with the given weight, updating shortest\n"
            "paths incrementally.\n\n"
            "C++: sssp.addEdge(v, w, weight, collectVertices, affectedVertices&)\n\n"
            "Returns the list of vertices whose shortest-path distance\n"
            "changed (empty list if collect_vertices is False).\n"
            "(out-param converted to a return value)")

        .def("removeEdge", &sssp_remove_edge,
            py::arg("v"), py::arg("w"), py::arg("collect_vertices") = false,
            "Remove edge (v, w), updating shortest paths incrementally.\n\n"
            "C++: sssp.removeEdge(v, w, collectVertices, affectedVertices&)\n\n"
            "Returns the list of vertices whose shortest-path distance\n"
            "changed (empty list if collect_vertices is False).\n"
            "(out-param converted to a return value)")

        .def("getShortestPathCost", &ompl::DynamicSSSP::getShortestPathCost, py::arg("u"),
            "Return the current shortest-path cost from the source to u.\n"
            "C++: sssp.getShortestPathCost(u)")

        .def("getShortestPathParent", &ompl::DynamicSSSP::getShortestPathParent, py::arg("u"),
            "Return the parent of u on the current shortest-path tree.\n"
            "C++: sssp.getShortestPathParent(u)");
}