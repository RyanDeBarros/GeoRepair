#include "MeshData.h"

#include <igl/adjacency_list.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/facet_adjacency_matrix.h>
#include <igl/invert_diag.h>
#include <igl/boundary_loop.h>

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

const decltype(MeshAuxiliaryData::boundary_loops)& MeshAuxiliaryData::get_boundary_loops(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::BOUNDARY_LOOPS))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::BOUNDARY_LOOPS);
		igl::boundary_loop(data.F, boundary_loops);
	}
	return boundary_loops;
}

const decltype(MeshAuxiliaryData::boundary_vertices)& MeshAuxiliaryData::get_boundary_vertices(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::BOUNDARY_VERTICES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::BOUNDARY_VERTICES);
		boundary_vertices.clear();
		const auto& loops = get_boundary_loops(data);
		for (const auto& loop : loops)
		{
			for (Eigen::Index v : loop)
				boundary_vertices.insert(v);
		}
	}
	return boundary_vertices;
}

const decltype(MeshAuxiliaryData::VN)& MeshAuxiliaryData::get_vertex_normals(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::VN))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::VN);
		igl::per_vertex_normals(data.V, data.F, VN);
	}
	return VN;
}

const decltype(MeshAuxiliaryData::laplacian)& MeshAuxiliaryData::get_laplacian(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::LAPLACIAN))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::LAPLACIAN);
		igl::cotmatrix(data.V, data.F, laplacian);
		
		Eigen::VectorXd diag = laplacian.diagonal();
		for (Eigen::Index i = 0; i < diag.rows(); ++i)
			if (diag(i) == 0) diag(0) = 1.0; // isolated vertices

		Eigen::SparseMatrix<double> inv_degree(laplacian.rows(), laplacian.cols());
		inv_degree.setIdentity();
		laplacian = (inv_degree * diag.cwiseInverse().asDiagonal()) * laplacian;
	}
	return laplacian;
}

const decltype(MeshAuxiliaryData::laplacian_eval)& MeshAuxiliaryData::get_laplacian_eval(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::LAPLACIAN_EVAL))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::LAPLACIAN_EVAL);
		laplacian_eval = get_laplacian(data) * data.V;
	}
	return laplacian_eval;
}

const decltype(MeshAuxiliaryData::laplacian_residuals)& MeshAuxiliaryData::get_laplacian_residuals(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::LAPLACIAN_RESIDUALS))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::LAPLACIAN_RESIDUALS);
		laplacian_residuals = get_laplacian_eval(data).rowwise().norm();
	}
	return laplacian_residuals;
}

const decltype(MeshAuxiliaryData::mass)& MeshAuxiliaryData::get_mass(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::MASS))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::MASS);
		igl::massmatrix(data.V, data.F, igl::MASSMATRIX_TYPE_VORONOI, mass);
	}
	return mass;
}

const decltype(MeshAuxiliaryData::mean_curvatures)& MeshAuxiliaryData::get_mean_curvatures(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::MEAN_CURVATURES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::MEAN_CURVATURES);
		Eigen::SparseMatrix<double> mass_inverse;
		igl::invert_diag(get_mass(data), mass_inverse);
		mean_curvatures = -mass_inverse * get_laplacian_eval(data);
	}
	return mean_curvatures;
}

const decltype(MeshAuxiliaryData::mean_curvature_magnitudes)& MeshAuxiliaryData::get_mean_curvature_magnitudes(const MeshPrimaryData& data)
{
	if (!auxiliary_flags.test((size_t)AuxiliaryFlags::MEAN_CURVATURE_MAGNITUDES))
	{
		auxiliary_flags.set((size_t)AuxiliaryFlags::MEAN_CURVATURE_MAGNITUDES);
		mean_curvature_magnitudes = get_mean_curvatures(data).rowwise().norm();
	}
	return mean_curvature_magnitudes;
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
