#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/adjacency_list.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/facet_adjacency_matrix.h>

#include <unordered_set>

// TODO disclaimer that triangulation will occur, and no information (for now, anyways) about texture coordinates, materials, etc., will be retained.

bool MeshData::load(const char* filename)
{
	// TODO eventually, add support for texture coordinates, etc., for specific file extensions
	if (!igl::read_triangle_mesh(filename, V, F))
		return false;
	refresh_data_structures();
	return true;
}

bool MeshData::save(const char* filename)
{
	return igl::write_triangle_mesh(filename, V, F);
}

void MeshData::refresh_data_structures()
{
	// TODO later, implement lazy load system to compute data structures only when needed.
	tree.init(V, F);
	igl::adjacency_list(F, VADJ);
	igl::vertex_triangle_adjacency(V.rows(), F, VFADJ_row, VFADJ_col);
	igl::facet_adjacency_matrix(F, FADJ);
	compute_connected_submeshes();
}

static void traverse_submesh_dfs(const Eigen::SparseMatrix<int>& fadj, Eigen::Index face_index, std::vector<bool>& visited_faces, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process)
{
	visited_faces[face_index] = true;
	for (Eigen::SparseMatrix<int>::InnerIterator iter(fadj, face_index); iter; ++iter)
	{
		Eigen::Index neighbour_face = iter.index();
		if (!visited_faces[neighbour_face])
		{
			adj_process(face_index, neighbour_face);
			traverse_submesh_dfs(fadj, neighbour_face, visited_faces, adj_process);
		}
	}
}

void MeshData::traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const
{
	std::vector<bool> visited_faces(F.rows(), false);
	for (Eigen::Index i = 0; i < F.rows(); ++i)
	{
		if (!visited_faces[i])
		{
			root_process(i);
			traverse_submesh_dfs(FADJ, i, visited_faces, adj_process);
		}
	}
}

void MeshData::traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const
{
	std::vector<bool> visited_faces(F.rows(), false);
	traverse_submesh_dfs(FADJ, submesh_root, visited_faces, adj_process);
}

static void connected_submesh_dfs(const Eigen::SparseMatrix<int>& fadj, Eigen::Index face_index, std::vector<bool>& visited_faces, std::vector<Eigen::Index>& submesh_faces)
{
	visited_faces[face_index] = true;
	submesh_faces.push_back(face_index);

	for (Eigen::SparseMatrix<int>::InnerIterator iter(fadj, face_index); iter; ++iter)
	{
		Eigen::Index neighbour_face = iter.index();
		if (!visited_faces[neighbour_face])
			connected_submesh_dfs(fadj, neighbour_face, visited_faces, submesh_faces);
	}
}

void MeshData::compute_connected_submeshes()
{
	connected_submeshes.clear();
	connected_submesh_roots.clear();
	std::vector<bool> visited_faces(F.rows(), false);
	size_t j = 0;
	for (Eigen::Index i = 0; i < F.rows(); ++i)
	{
		if (!visited_faces[i])
		{
			std::vector<Eigen::Index> submesh_faces;
			connected_submesh_roots[i] = j++;
			connected_submesh_dfs(FADJ, i, visited_faces, submesh_faces);
			connected_submeshes.push_back(submesh_faces);
		}
	}
}
