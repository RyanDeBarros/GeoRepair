#pragma once

#include <igl/opengl/ViewerData.h>

#include <bitset>

struct MeshPrimaryData
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
};

// used for lazy loading auxiliary data structures
class MeshAuxiliaryData
{
	enum class AuxiliaryFlags
	{
		VADJ,
		VFADJ,
		FADJ,
		CONNECTED_SUBMESHES,
		_COUNT
	};
	std::bitset<(size_t)AuxiliaryFlags::_COUNT> auxiliary_flags;

	std::vector<std::vector<Eigen::Index>> VADJ;
	std::vector<std::vector<Eigen::Index>> VFADJ_row;
	std::vector<std::vector<Eigen::Index>> VFADJ_col;
	Eigen::SparseMatrix<int> FADJ;
	std::vector<std::vector<Eigen::Index>> connected_submeshes;
	std::unordered_map<Eigen::Index, size_t> connected_submesh_roots;

public:
	void reset();
	const decltype(VADJ)& get_vadj(const MeshPrimaryData&);
	const decltype(VFADJ_row)& get_vfadj_row(const MeshPrimaryData&);
	const decltype(VFADJ_col)& get_vfadj_col(const MeshPrimaryData&);
	const decltype(FADJ)& get_fadj(const MeshPrimaryData&);
	const decltype(connected_submeshes)& get_connected_submeshes(const MeshPrimaryData&);
	const decltype(connected_submesh_roots)& get_connected_submesh_roots(const MeshPrimaryData&);

private:
	void compute_connected_submeshes(const MeshPrimaryData&);
};
