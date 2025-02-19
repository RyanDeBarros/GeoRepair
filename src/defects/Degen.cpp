#include "DegenerateFaces.h"

bool defects::DegenerateFaces::detect(const MeshData& mesh)
{
	reset();
	const auto& vertices = mesh.get_vertices();
	const auto& faces = mesh.get_faces();
	std::vector<std::vector<Eigen::Index>> vertex_groups;
	vertex_classes.clear();
	for (Eigen::Index i = 0; i < faces.rows(); ++i)
	{
		Eigen::Index f0 = faces(i, 0);
		Eigen::Index f1 = faces(i, 1);
		Eigen::Index f2 = faces(i, 2);
		Eigen::RowVector3d v0 = vertices.row(f0);
		Eigen::RowVector3d v1 = vertices.row(f1);
		Eigen::RowVector3d v2 = vertices.row(f2);
		if (area(v0, v1, v2) <= acceptance)
		{
			if (degenerate_face_indices.empty())
				face_maximum_block_height = i;
			else
				face_maximum_block_height = std::max(face_maximum_block_height, i - degenerate_face_indices.back() - 1);
			degenerate_face_indices.push_back(i);
			vertex_classes.add({ f0, f1, f2 });
		}
	}
	face_maximum_block_height = std::max(face_maximum_block_height, faces.rows() - degenerate_face_indices.back() - 1);
	return !degenerate_face_indices.empty();
}

void defects::DegenerateFaces::repair(MeshData& mesh)
{
	if (degenerate_face_indices.empty())
		return;
	
	auto& vertices = mesh.get_vertices();
	auto& faces = mesh.get_faces();
	std::vector<Eigen::Index> remove_vertex_indices;
	auto classes = vertex_classes.gen_classes();
	for (const auto& [root, group] : classes)
	{
		Eigen::RowVector3d mean(0.0, 0.0, 0.0);
		for (auto v : group)
		{
			mean += vertices.row(v);
			if (v != root)
			{
				remove_vertex_indices.push_back(v);
				// search for all faces that use vertices in degenerate faces, and replace with vertex root
				const auto& r = mesh.get_adj_face_rows(v);
				const auto& c = mesh.get_adj_face_cols(v);
				for (Eigen::Index i = 0; i < r.size(); ++i)
					faces(r[i], c[i]) = root;
			}
		}
		mean *= 1.0 / group.size();
		vertices.row(root) = mean; // collapse vertices to mean point
	}

	remove_rows(faces, degenerate_face_indices, face_maximum_block_height, true);
	Eigen::Index vmax = -1, vmin = vertices.rows();
	for (Eigen::Index v : remove_vertex_indices)
	{
		vmax = std::max(vmax, v);
		vmin = std::min(vmin, v);
	}
	Eigen::Index vmax_block_height = 0;
	vmax_block_height = std::max(vmax_block_height, vmin);
	vmax_block_height = std::max(vmax_block_height, vmax - vmin - 1);
	vmax_block_height = std::max(vmax_block_height, vertices.rows() - vmax - 1);
	remove_rows(vertices, remove_vertex_indices, vmax_block_height, false);
	// TODO iterate through all faces and remove those that have duplicate vertex indices
	reindex_faces(faces, remove_vertex_indices);
	reset();
}

void defects::DegenerateFaces::reset()
{
	degenerate_face_indices.clear();
	vertex_classes.clear();
	face_maximum_block_height = 0;
}
