#include "UnpatchedHoles.h"

// TODO split concave n-gons into convex sub-n-gons so there's no z-fighting with new faces

void defects::UnpatchedHoles::_detect(const Mesh& mesh)
{
	boundary_vertices = mesh.get_boundary_loops();
}

void defects::UnpatchedHoles::_repair(Mesh& mesh)
{
	add_faces.clear();
	add_vertices.clear();
	void(defects::UnpatchedHoles::*repair_func)(Mesh&, const std::vector<Eigen::Index>&, bool) = nullptr;

	switch (patch_method)
	{
	case PatchMethod::FAN:
		repair_func = &defects::UnpatchedHoles::repair_fan;
		break;
	case PatchMethod::STRIP:
		repair_func = &defects::UnpatchedHoles::repair_strip;
		break;
	case PatchMethod::CLIP:
		repair_func = &defects::UnpatchedHoles::repair_clip;
		break;
	case PatchMethod::PIE:
		repair_func = &defects::UnpatchedHoles::repair_pie;
		break;
	}
	
	const auto& faces = mesh.get_faces();
	for (const auto& boundary : boundary_vertices)
	{
		Eigen::Index v0 = boundary[0], v1 = boundary[1];
		bool increasing = true;
		for (Eigen::Index i = 0; i < faces.rows(); ++i)
		{
			const Eigen::RowVector3i& face = faces.row(i);
			// note that increasing should be the opposite of whether boundary faces use increasing winding order
			if (v0 == face(0))
			{
				if (v1 == face(1))
				{
					increasing = false;
					break;
				}
				else if (v1 == face(2))
				{
					increasing = true;
					break;
				}
			}
			else if (v0 == face(1))
			{
				if (v1 == face(2))
				{
					increasing = false;
					break;
				}
				else if (v1 == face(0))
				{
					increasing = true;
					break;
				}
			}
			else if (v0 == face(2))
			{
				if (v1 == face(0))
				{
					increasing = false;
					break;
				}
				else if (v1 == face(1))
				{
					increasing = true;
					break;
				}
			}
		}
		(this->*repair_func)(mesh, boundary, increasing);
	}

	if (!add_faces.empty())
	{
		auto& faces = mesh.get_faces();
		Eigen::MatrixXi new_faces(faces.rows() + add_faces.size(), faces.cols());
		new_faces.topRows(faces.rows()) = faces;
		for (size_t i = 0; i < add_faces.size(); ++i)
			new_faces.row(faces.rows() + i) = add_faces[i];
		std::swap(faces, new_faces);
	}

	if (!add_vertices.empty())
	{
		auto& vertices = mesh.get_vertices();
		Eigen::MatrixXd new_vertices(vertices.rows() + add_vertices.size(), vertices.cols());
		new_vertices.topRows(vertices.rows()) = vertices;
		for (size_t i = 0; i < add_vertices.size(); ++i)
			new_vertices.row(vertices.rows() + i) = add_vertices[i];
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

void defects::UnpatchedHoles::repair_fan(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing)
{
	for (size_t i = 2; i < boundary.size(); ++i)
	{
		Eigen::RowVector3i face(boundary[0], boundary[i - 1], boundary[i]);
		add_faces.push_back(increasing ? face : face.reverse());
	}
}

void defects::UnpatchedHoles::repair_strip(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing)
{
	int i = 0;
	int j = boundary.size() - 1;
	bool forward = true;
	while (true)
	{
		if (forward)
		{
			if (j <= i + 1)
				break;
			Eigen::RowVector3i face(boundary[i], boundary[i + 1], boundary[j]);
			add_faces.push_back(increasing ? face : face.reverse());
			++i;
			forward = false;
		}
		else
		{
			if (j - 1 <= i)
				break;
			Eigen::RowVector3i face(boundary[i], boundary[j - 1], boundary[j]);
			add_faces.push_back(increasing ? face : face.reverse());
			--j;
			forward = true;
		}
	}
}

void defects::UnpatchedHoles::repair_clip(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing)
{
	size_t j = 1;
	size_t k = 2;
	do
	{
		for (size_t i = 0; i < boundary.size() - 1; i += k)
		{
			if (i + k < boundary.size())
			{
				Eigen::RowVector3i face(boundary[i], boundary[i + j], boundary[i + k]);
				add_faces.push_back(increasing ? face : face.reverse());
			}
			else if (i + k == boundary.size() && i != 0)
			{
				Eigen::RowVector3i face(boundary[i], boundary[i + j], boundary[0]);
				add_faces.push_back(increasing ? face : face.reverse());
			}
			else
				break;
		}
		j = k;
		k = j << 1;
	} while (k < boundary.size());
}

void defects::UnpatchedHoles::repair_pie(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing)
{
	const auto& vertices = mesh.get_vertices();
	Eigen::RowVector3d mean_vertex(0.0, 0.0, 0.0);
	for (auto v : boundary)
		mean_vertex += vertices.row(v);
	mean_vertex /= boundary.size();
	Eigen::Index mean_index = add_vertices.size() + vertices.rows();
	add_vertices.push_back(mean_vertex);

	for (size_t i = 0; i < boundary.size(); ++i)
	{
		Eigen::RowVector3i face(mean_index, boundary[i], boundary[(i + 1) % boundary.size()]);
		add_faces.push_back(increasing ? face : face.reverse());
	}
}
