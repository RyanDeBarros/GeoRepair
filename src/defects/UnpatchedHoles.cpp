#include "UnpatchedHoles.h"

#include <igl/boundary_loop.h>

void defects::UnpatchedHoles::_detect(const MeshData& mesh)
{
	igl::boundary_loop(mesh.get_faces(), boundary_vertices);
}

void defects::UnpatchedHoles::_repair(MeshData& mesh)
{
	add_faces.clear();
	add_vertices.clear();
	void(defects::UnpatchedHoles::*repair_func)(MeshData&, const std::vector<Eigen::Index>&) = nullptr;

	switch (patch_method)
	{
	case PatchMethod::FAN:
		repair_func = repair_fan;
		break;
	case PatchMethod::STRIP:
		repair_func = repair_strip;
		break;
	case PatchMethod::CLIP:
		repair_func = repair_clip;
		break;
	case PatchMethod::PIE:
		repair_func = repair_pie;
		break;
	}

	for (const auto& boundary : boundary_vertices)
		(this->*repair_func)(mesh, boundary);

	if (!add_faces.empty())
	{
		auto& faces = mesh.get_faces();
		Eigen::MatrixXi new_faces(faces.rows() + add_faces.size(), faces.cols());
		new_faces << faces;
		for (auto face : add_faces)
			new_faces << face;
		std::swap(faces, new_faces);
	}

	if (!add_vertices.empty())
	{
		auto& vertices = mesh.get_vertices();
		Eigen::MatrixXd new_vertices(vertices.rows() + add_vertices.size(), vertices.cols());
		new_vertices << vertices;
		for (auto vertex : add_vertices)
			new_vertices << vertex;
		std::swap(vertices, new_vertices);
	}
}

void defects::UnpatchedHoles::reset()
{
	boundary_vertices.clear();
}

bool defects::UnpatchedHoles::in_detected_state() const
{
	return !boundary_vertices.empty();
}

void defects::UnpatchedHoles::repair_fan(MeshData& mesh, const std::vector<Eigen::Index>& boundary)
{
	for (size_t i = 2; i < boundary.size(); ++i)
	{
		Eigen::RowVector3i face(boundary[0], boundary[i - 1], boundary[i]);
		add_faces.push_back(face);
	}
}

void defects::UnpatchedHoles::repair_strip(MeshData& mesh, const std::vector<Eigen::Index>& boundary)
{
	// TODO
}

void defects::UnpatchedHoles::repair_clip(MeshData& mesh, const std::vector<Eigen::Index>& boundary)
{
	// TODO
}

void defects::UnpatchedHoles::repair_pie(MeshData& mesh, const std::vector<Eigen::Index>& boundary)
{
	const auto& vertices = mesh.get_vertices();
	Eigen::RowVector3d mean_vertex(0.0, 0.0, 0.0);
	for (auto v : boundary)
		mean_vertex += vertices.row(v);
	mean_vertex /= boundary.size();
	Eigen::Index mean_index = add_vertices.size() + vertices.rows();
	add_vertices.push_back(mean_vertex);

	for (size_t i = 0; i < boundary.size() - 1; ++i)
	{
		Eigen::RowVector3i face(mean_index, boundary[i], boundary[i + 1]);
		add_faces.push_back(face);
	}
}
