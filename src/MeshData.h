#pragma once

#include <igl/opengl/ViewerData.h>
#include <igl/AABB.h>

class MeshData
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	std::vector<std::vector<Eigen::Index>> VADJ;
	std::vector<std::vector<Eigen::Index>> VFADJ_row;
	std::vector<std::vector<Eigen::Index>> VFADJ_col;
	Eigen::SparseMatrix<int> FADJ;
	igl::AABB<Eigen::MatrixXd, 3> tree;
	std::vector<std::vector<Eigen::Index>> connected_submeshes;
	std::unordered_map<Eigen::Index, size_t> connected_submesh_roots;

public:
	bool load(const char* filename);
	bool save(const char* filename);
	void refresh_data_structures();

	const decltype(V)& get_vertices() const { return V; }
	decltype(V)& get_vertices() { return V; }
	const decltype(F)& get_faces() const { return F; }
	decltype(F)& get_faces() { return F; }
	const std::vector<Eigen::Index>& get_adj_vertices(Eigen::Index vertex_index) const { return VADJ[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_rows(Eigen::Index vertex_index) const { return VFADJ_row[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_cols(Eigen::Index vertex_index) const { return VFADJ_col[vertex_index]; }
	Eigen::SparseMatrix<int>::InnerIterator get_adj_face_iterator(Eigen::Index face_index) const { return Eigen::SparseMatrix<int>::InnerIterator(FADJ, face_index); }
	const decltype(tree)& get_tree() const { return tree; }
	const decltype(connected_submeshes)& get_connected_submeshes() const { return connected_submeshes; }

	size_t connected_submesh_index(Eigen::Index submesh_root) const { return connected_submesh_roots.find(submesh_root)->second; }
	const std::vector<Eigen::Index>& get_connected_submesh(Eigen::Index submesh_root) const { return connected_submeshes[connected_submesh_index(submesh_root)]; }
	
	void traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
	void traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;

private:
	void compute_connected_submeshes();
};
