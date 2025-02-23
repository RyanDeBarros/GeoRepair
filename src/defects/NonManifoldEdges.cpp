#include "NonManifoldEdges.h"

void defects::NonManifoldEdges::_detect(const Mesh& mesh)
{
    const auto& faces = mesh.get_faces();
    for (Eigen::Index i = 0; i < faces.rows(); ++i)
    {
        const auto& face = faces.row(i);
        add_undirected_edge(face(0), face(1));
        add_undirected_edge(face(1), face(2));
        add_undirected_edge(face(2), face(0));
    }
}

void defects::NonManifoldEdges::_repair(Mesh& mesh)
{
    // nothing to repair, as there is no obvious general solution to fixing non-manifold edges, which might even be intentional.
    // TODO disclaimer of the above comment
}

void defects::NonManifoldEdges::reset()
{
    non_manifold_edges.clear();
    edge_counts.clear();
}

bool defects::NonManifoldEdges::in_detected_state() const
{
    return !non_manifold_edges.empty();
}

void defects::NonManifoldEdges::add_undirected_edge(Eigen::Index v1, Eigen::Index v2)
{
    UndirectedEdge edge{ v1, v2 };
    auto it = edge_counts.find(edge);
    if (it != edge_counts.end())
    {
        ++it->second;
        if (it->second > 2)
            non_manifold_edges.insert(edge);
    }
    else
        edge_counts[edge] = 1;
}

void defects::NonManifoldEdges::traverse_non_manifold_edges(const std::function<void(Eigen::Index v1, Eigen::Index v2, size_t count)>& process) const
{
    for (auto iter = non_manifold_edges.begin(); iter != non_manifold_edges.end(); ++iter)
        process(iter->v1, iter->v2, edge_counts.find(*iter)->second);
}
