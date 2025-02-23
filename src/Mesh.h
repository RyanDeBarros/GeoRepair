#pragma once

#include "History.h"

class Mesh
{
	History history;
	std::shared_ptr<MeshPrimaryData> data;
	mutable MeshAuxiliaryData aux;

public:
	Mesh() : data(std::make_shared<MeshPrimaryData>()) {}

	bool load(const char* filename);
	bool save(const char* filename);

	void push();
	void undo();
	void redo();

	const decltype(data->V)& get_vertices() const { return data->V; }
	decltype(data->V)& get_vertices() { return data->V; }
	const decltype(data->F)& get_faces() const { return data->F; }
	decltype(data->F)& get_faces() { return data->F; }

	const std::vector<Eigen::Index>& get_adj_vertices(Eigen::Index vertex_index) const { return aux.get_vadj(*data)[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_rows(Eigen::Index vertex_index) const { return aux.get_vfadj_row(*data)[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_cols(Eigen::Index vertex_index) const { return aux.get_vfadj_col(*data)[vertex_index]; }
	Eigen::SparseMatrix<int>::InnerIterator get_adj_face_iterator(Eigen::Index face_index) const { return Eigen::SparseMatrix<int>::InnerIterator(aux.get_fadj(*data), face_index); }
	const std::vector<std::vector<Eigen::Index>>& get_connected_submeshes() const { return aux.get_connected_submeshes(*data); }
	size_t connected_submesh_index(Eigen::Index submesh_root) const { return aux.get_connected_submesh_roots(*data).find(submesh_root)->second; }
	const std::vector<Eigen::Index>& get_connected_submesh(Eigen::Index submesh_root) const { return aux.get_connected_submeshes(*data)[connected_submesh_index(submesh_root)]; }

	void traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
	void traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
};

