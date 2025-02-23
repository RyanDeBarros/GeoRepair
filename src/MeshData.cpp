#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/adjacency_list.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/facet_adjacency_matrix.h>

// TODO disclaimer that triangulation will occur, and no information (for now, anyways) about texture coordinates, materials, etc., will be retained.

void MeshAuxiliaryData::reset()
{
	auxiliary_flags.reset();
}

const decltype(MeshAuxiliaryData::VADJ)& MeshAuxiliaryData::get_vadj(const MeshData& mesh)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VADJ);
		igl::adjacency_list(mesh.get_faces(), VADJ);
	}
	return VADJ;
}

const decltype(MeshAuxiliaryData::VFADJ_row)& MeshAuxiliaryData::get_vfadj_row(const MeshData& mesh)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VFADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VFADJ);
		igl::vertex_triangle_adjacency(mesh.get_vertices().rows(), mesh.get_faces(), VFADJ_row, VFADJ_col);
	}
	return VFADJ_row;
}

const decltype(MeshAuxiliaryData::VFADJ_col)& MeshAuxiliaryData::get_vfadj_col(const MeshData& mesh)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VFADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VFADJ);
		igl::vertex_triangle_adjacency(mesh.get_vertices().rows(), mesh.get_faces(), VFADJ_row, VFADJ_col);
	}
	return VFADJ_col;
}

const decltype(MeshAuxiliaryData::FADJ)& MeshAuxiliaryData::get_fadj(const MeshData& mesh)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::FADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::FADJ);
		igl::facet_adjacency_matrix(mesh.get_faces(), FADJ);
	}
	return FADJ;
}

const decltype(MeshAuxiliaryData::connected_submeshes)& MeshAuxiliaryData::get_connected_submeshes(const MeshData& mesh)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES);
		compute_connected_submeshes(mesh);
	}
	return connected_submeshes;
}

const decltype(MeshAuxiliaryData::connected_submesh_roots)& MeshAuxiliaryData::get_connected_submesh_roots(const MeshData& mesh)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES);
		compute_connected_submeshes(mesh);
	}
	return connected_submesh_roots;
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

void MeshAuxiliaryData::compute_connected_submeshes(const MeshData& mesh)
{
	connected_submeshes.clear();
	connected_submesh_roots.clear();
	const auto& fadj = get_fadj(mesh);
	std::vector<bool> visited_faces(mesh.get_faces().rows(), false);
	size_t j = 0;
	for (Eigen::Index i = 0; i < visited_faces.size(); ++i)
	{
		if (!visited_faces[i])
		{
			std::vector<Eigen::Index> submesh_faces;
			connected_submesh_roots[i] = j++;
			connected_submesh_dfs(fadj, i, visited_faces, submesh_faces);
			connected_submeshes.push_back(submesh_faces);
		}
	}
}

bool MeshData::load(const char* filename)
{
	// TODO eventually, add support for texture coordinates, etc., for specific file extensions
	if (!igl::read_triangle_mesh(filename, V, F))
		return false;
	refresh_auxiliary();
	return true;
}

bool MeshData::save(const char* filename)
{
	return igl::write_triangle_mesh(filename, V, F);
}

void MeshData::refresh_auxiliary()
{
	aux.reset();
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
	const auto& FADJ = aux.get_fadj(*this);
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
	traverse_submesh_dfs(aux.get_fadj(*this), submesh_root, visited_faces, adj_process);
}
