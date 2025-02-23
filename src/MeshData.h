#pragma once

#include <igl/opengl/ViewerData.h>

#include <bitset>

class MeshData;

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
	const decltype(VADJ)& get_vadj(const MeshData&);
	const decltype(VFADJ_row)& get_vfadj_row(const MeshData&);
	const decltype(VFADJ_col)& get_vfadj_col(const MeshData&);
	const decltype(FADJ)& get_fadj(const MeshData&);
	const decltype(connected_submeshes)& get_connected_submeshes(const MeshData&);
	const decltype(connected_submesh_roots)& get_connected_submesh_roots(const MeshData&);

private:
	void compute_connected_submeshes(const MeshData&);
};

class MeshData
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	mutable MeshAuxiliaryData aux;

public:
	bool load(const char* filename);
	bool save(const char* filename);
	void refresh_auxiliary();

	const decltype(V)& get_vertices() const { return V; }
	decltype(V)& get_vertices() { return V; }
	const decltype(F)& get_faces() const { return F; }
	decltype(F)& get_faces() { return F; }

	const std::vector<Eigen::Index>& get_adj_vertices(Eigen::Index vertex_index) const { return aux.get_vadj(*this)[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_rows(Eigen::Index vertex_index) const { return aux.get_vfadj_row(*this)[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_cols(Eigen::Index vertex_index) const { return aux.get_vfadj_col(*this)[vertex_index]; }
	Eigen::SparseMatrix<int>::InnerIterator get_adj_face_iterator(Eigen::Index face_index) const { return Eigen::SparseMatrix<int>::InnerIterator(aux.get_fadj(*this), face_index); }
	const std::vector<std::vector<Eigen::Index>>& get_connected_submeshes() const { return aux.get_connected_submeshes(*this); }
	size_t connected_submesh_index(Eigen::Index submesh_root) const { return aux.get_connected_submesh_roots(*this).find(submesh_root)->second; }
	const std::vector<Eigen::Index>& get_connected_submesh(Eigen::Index submesh_root) const { return aux.get_connected_submeshes(*this)[connected_submesh_index(submesh_root)]; }
	
	void traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
	void traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
};
