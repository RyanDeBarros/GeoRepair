#pragma once

#include <igl/opengl/ViewerData.h>

#include <bitset>
#include <unordered_set>

struct MeshPrimaryData
{
	std::string mesh_filename;
	Eigen::MatrixXd V; // vertices
	Eigen::MatrixXi F; // faces
	Eigen::MatrixXd VC; // vertex colors
	Eigen::MatrixXd FC; // face colors
	Eigen::MatrixXd E1; // 1st vertices in edges
	Eigen::MatrixXd E2; // 2nd vertices in edges
	Eigen::MatrixXd EC; // edge colors
};

// used for lazy loading auxiliary data structures
class MeshAuxiliaryData
{
	enum class AuxiliaryFlags
	{
		VADJ,
		VFADJ,
		FADJ,
		BOUNDARY_LOOPS,
		BOUNDARY_VERTICES,
		VN,
		LAPLACIAN,
		LAPLACIAN_EVAL,
		LAPLACIAN_RESIDUALS,
		MASS,
		MEAN_CURVATURES,
		MEAN_CURVATURE_MAGNITUDES,
		CONNECTED_SUBMESHES,
		_COUNT
	};
	std::bitset<(size_t)AuxiliaryFlags::_COUNT> auxiliary_flags;

	std::vector<std::vector<Eigen::Index>> VADJ;
	std::vector<std::vector<Eigen::Index>> VFADJ_row;
	std::vector<std::vector<Eigen::Index>> VFADJ_col;
	Eigen::SparseMatrix<int> FADJ;
	std::vector<std::vector<Eigen::Index>> boundary_loops;
	std::unordered_set<Eigen::Index> boundary_vertices;
	Eigen::MatrixXd VN;
	Eigen::SparseMatrix<double> laplacian;
	Eigen::MatrixXd laplacian_eval;
	Eigen::VectorXd laplacian_residuals;
	Eigen::SparseMatrix<double> mass;
	Eigen::MatrixXd mean_curvatures;
	Eigen::VectorXd mean_curvature_magnitudes;
	std::vector<std::vector<Eigen::Index>> connected_submeshes;
	std::unordered_map<Eigen::Index, size_t> connected_submesh_roots;

public:
	void reset();
	const decltype(VADJ)& get_vadj(const MeshPrimaryData&);
	const decltype(VFADJ_row)& get_vfadj_row(const MeshPrimaryData&);
	const decltype(VFADJ_col)& get_vfadj_col(const MeshPrimaryData&);
	const decltype(FADJ)& get_fadj(const MeshPrimaryData&);
	const decltype(boundary_loops)& get_boundary_loops(const MeshPrimaryData&);
	const decltype(boundary_vertices)& get_boundary_vertices(const MeshPrimaryData&);
	const decltype(VN)& get_vertex_normals(const MeshPrimaryData&);
	const decltype(laplacian)& get_laplacian(const MeshPrimaryData&);
	const decltype(laplacian_eval)& get_laplacian_eval(const MeshPrimaryData&);
	const decltype(laplacian_residuals)& get_laplacian_residuals(const MeshPrimaryData&);
	const decltype(mass)& get_mass(const MeshPrimaryData&);
	const decltype(mean_curvatures)& get_mean_curvatures(const MeshPrimaryData&);
	const decltype(mean_curvature_magnitudes)& get_mean_curvature_magnitudes(const MeshPrimaryData&);
	const decltype(connected_submeshes)& get_connected_submeshes(const MeshPrimaryData&);
	const decltype(connected_submesh_roots)& get_connected_submesh_roots(const MeshPrimaryData&);

private:
	void compute_connected_submeshes(const MeshPrimaryData&);
};

class MeshData
{
	mutable MeshAuxiliaryData aux;

public:
	MeshPrimaryData primary;

	MeshData() = default;
	MeshData(const MeshData& other) : primary(other.primary) {}
	MeshData& operator=(const MeshData& other)
	{
		if (this != &other)
		{
			primary = other.primary;
			aux.reset();
		}
		return *this;
	}

	void reset_aux() const { aux.reset(); }
	const std::vector<std::vector<Eigen::Index>>& get_vadj() const { return aux.get_vadj(primary); }
	const std::vector<std::vector<Eigen::Index>>& get_vfadj_row() const { return aux.get_vfadj_row(primary); }
	const std::vector<std::vector<Eigen::Index>>& get_vfadj_col() const { return aux.get_vfadj_col(primary); }
	const Eigen::SparseMatrix<int>& get_fadj() const { return aux.get_fadj(primary); }
	const std::vector<std::vector<Eigen::Index>>& get_boundary_loops() const { return aux.get_boundary_loops(primary); }
	const std::unordered_set<Eigen::Index>& get_boundary_vertices() const { return aux.get_boundary_vertices(primary); }
	const Eigen::MatrixXd& get_vertex_normals() const { return aux.get_vertex_normals(primary); }
	const Eigen::SparseMatrix<double>& get_laplacian() const { return aux.get_laplacian(primary); }
	const Eigen::MatrixXd& get_laplacian_eval() const { return aux.get_laplacian_eval(primary); }
	const Eigen::VectorXd& get_laplacian_residuals() const { return aux.get_laplacian_residuals(primary); }
	const Eigen::SparseMatrix<double>& get_mass() const { return aux.get_mass(primary); }
	const Eigen::MatrixXd& get_mean_curvatures() const { return aux.get_mean_curvatures(primary); }
	const Eigen::VectorXd& get_mean_curvature_magnitudes() const { return aux.get_mean_curvature_magnitudes(primary); }
	const std::vector<std::vector<Eigen::Index>>& get_connected_submeshes() const { return aux.get_connected_submeshes(primary); }
	const std::unordered_map<Eigen::Index, size_t>& get_connected_submesh_roots() const { return aux.get_connected_submesh_roots(primary); }
};
