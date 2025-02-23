#include "MeshData.h"

#include <igl/adjacency_list.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/facet_adjacency_matrix.h>

// TODO disclaimer that triangulation will occur, and no information (for now, anyways) about texture coordinates, materials, etc., will be retained.

void MeshAuxiliaryData::reset()
{
	auxiliary_flags.reset();
}

const decltype(MeshAuxiliaryData::VADJ)& MeshAuxiliaryData::get_vadj(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VADJ);
		igl::adjacency_list(data.F, VADJ);
	}
	return VADJ;
}

const decltype(MeshAuxiliaryData::VFADJ_row)& MeshAuxiliaryData::get_vfadj_row(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VFADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VFADJ);
		igl::vertex_triangle_adjacency(data.V.rows(), data.F, VFADJ_row, VFADJ_col);
	}
	return VFADJ_row;
}

const decltype(MeshAuxiliaryData::VFADJ_col)& MeshAuxiliaryData::get_vfadj_col(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VFADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VFADJ);
		igl::vertex_triangle_adjacency(data.V.rows(), data.F, VFADJ_row, VFADJ_col);
	}
	return VFADJ_col;
}

const decltype(MeshAuxiliaryData::FADJ)& MeshAuxiliaryData::get_fadj(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::FADJ))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::FADJ);
		igl::facet_adjacency_matrix(data.F, FADJ);
	}
	return FADJ;
}

const decltype(MeshAuxiliaryData::connected_submeshes)& MeshAuxiliaryData::get_connected_submeshes(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES);
		compute_connected_submeshes(data);
	}
	return connected_submeshes;
}

const decltype(MeshAuxiliaryData::connected_submesh_roots)& MeshAuxiliaryData::get_connected_submesh_roots(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::CONNECTED_SUBMESHES);
		compute_connected_submeshes(data);
	}
	return connected_submesh_roots;
}

static void connected_submesh_dfs(const Eigen::SparseMatrix<int>& fadj, Eigen::Index face_index, std::vector<bool>& visited_faces, std::vector<Eigen::Index>& subdata_faces)
{
	visited_faces[face_index] = true;
	subdata_faces.push_back(face_index);

	for (Eigen::SparseMatrix<int>::InnerIterator iter(fadj, face_index); iter; ++iter)
	{
		Eigen::Index neighbour_face = iter.index();
		if (!visited_faces[neighbour_face])
			connected_submesh_dfs(fadj, neighbour_face, visited_faces, subdata_faces);
	}
}

void MeshAuxiliaryData::compute_connected_submeshes(const MeshPrimaryData& data)
{
	connected_submeshes.clear();
	connected_submesh_roots.clear();
	const auto& fadj = get_fadj(data);
	std::vector<bool> visited_faces(data.F.rows(), false);
	size_t j = 0;
	for (Eigen::Index i = 0; i < visited_faces.size(); ++i)
	{
		if (!visited_faces[i])
		{
			std::vector<Eigen::Index> subdata_faces;
			connected_submesh_roots[i] = j++;
			connected_submesh_dfs(fadj, i, visited_faces, subdata_faces);
			connected_submeshes.push_back(subdata_faces);
		}
	}
}
