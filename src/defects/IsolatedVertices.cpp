#include "IsolatedVertices.h"

void defects::IsolatedVertices::_detect(const Mesh& mesh)
{
	std::vector<bool> vertex_has_face(mesh.get_vertices().rows(), false);
	const auto& faces = mesh.get_faces();
	for (Eigen::Index i = 0; i < faces.rows(); ++i)
	{
		vertex_has_face[faces(i, 0)] = true;
		vertex_has_face[faces(i, 1)] = true;
		vertex_has_face[faces(i, 2)] = true;
	}
	for (Eigen::Index i = 0; i < vertex_has_face.size(); ++i)
		if (!vertex_has_face[i])
			isolated_vertices.push_back(i);
}

void defects::IsolatedVertices::_repair(Mesh& mesh)
{
	remove_rows(mesh.get_vertices(), isolated_vertices, true);
	reindex_faces(mesh.get_faces(), isolated_vertices);
	mesh.reset_vertex_colors();
}

void defects::IsolatedVertices::reset()
{
	isolated_vertices.clear();
}

bool defects::IsolatedVertices::in_detected_state() const
{
	return !isolated_vertices.empty();
}
