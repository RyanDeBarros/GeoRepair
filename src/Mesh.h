#pragma once

#include "MeshHistory.h"

class Mesh
{
	MeshHistory history;
	std::shared_ptr<MeshPrimaryData> data;
	mutable MeshAuxiliaryData aux;

public:
	Mesh() : data(std::make_shared<MeshPrimaryData>()) {}

	bool load(const char* filename);
	bool save(const char* filename);

	void push();
	bool undo();
	bool redo();

	int history_index() const { return history.index(); }
	const std::string& get_filename() const { return data->mesh_filename; }

	const decltype(data->V)& get_vertices() const { return data->V; }
	decltype(data->V)& get_vertices() { return data->V; }
	const decltype(data->F)& get_faces() const { return data->F; }
	decltype(data->F)& get_faces() { return data->F; }
	const decltype(data->VC)& get_vertex_colors() const { return data->VC; }
	decltype(data->VC)& get_vertex_colors() { return data->VC; }
	const decltype(data->FC)& get_face_colors() const { return data->FC; }
	decltype(data->FC)& get_face_colors() { return data->FC; }
	const decltype(data->E1)& get_edges_1() const { return data->E1; }
	decltype(data->E1)& get_edges_1() { return data->E1; }
	const decltype(data->E1)& get_edges_2() const { return data->E2; }
	decltype(data->E1)& get_edges_2() { return data->E2; }
	const decltype(data->EC)& get_edge_colors() const { return data->EC; }
	decltype(data->EC)& get_edge_colors() { return data->EC; }

	void reset_vertex_colors();
	void reset_face_colors();
	void reset_edge_colors();

	const std::vector<Eigen::Index>& get_adj_vertices(Eigen::Index vertex_index) const { return aux.get_vadj(*data)[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_rows(Eigen::Index vertex_index) const { return aux.get_vfadj_row(*data)[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_cols(Eigen::Index vertex_index) const { return aux.get_vfadj_col(*data)[vertex_index]; }
	Eigen::SparseMatrix<int>::InnerIterator get_adj_face_iterator(Eigen::Index face_index) const { return Eigen::SparseMatrix<int>::InnerIterator(aux.get_fadj(*data), face_index); }
	const std::vector<std::vector<Eigen::Index>> get_boundary_loops() const { return aux.get_boundary_loops(*data); }
	const std::unordered_set<Eigen::Index> get_boundary_vertices() const { return aux.get_boundary_vertices(*data); }
	const Eigen::MatrixXd& get_vertex_normals() const { return aux.get_vertex_normals(*data); }
	const Eigen::SparseMatrix<double>& get_laplacian() const { return aux.get_laplacian(*data); }
	Eigen::SparseMatrix<double>::InnerIterator get_laplacian_iterator(Eigen::Index vertex_index) const { return Eigen::SparseMatrix<double>::InnerIterator(aux.get_laplacian(*data), vertex_index); }
	const Eigen::MatrixXd& get_laplacian_eval() const { return aux.get_laplacian_eval(*data); }
	const Eigen::VectorXd& get_laplacian_residuals() const { return aux.get_laplacian_residuals(*data); }
	Eigen::SparseMatrix<double>::InnerIterator get_mass_iterator(Eigen::Index vertex_index) const { return Eigen::SparseMatrix<double>::InnerIterator(aux.get_mass(*data), vertex_index); }
	const Eigen::MatrixXd& get_mean_curvatures() const { return aux.get_mean_curvatures(*data); }
	const Eigen::VectorXd& get_mean_curvature_magnitudes() const { return aux.get_mean_curvature_magnitudes(*data); }

	const std::vector<std::vector<Eigen::Index>>& get_connected_submeshes() const { return aux.get_connected_submeshes(*data); }
	size_t connected_submesh_index(Eigen::Index submesh_root) const { return aux.get_connected_submesh_roots(*data).find(submesh_root)->second; }
	const std::vector<Eigen::Index>& get_connected_submesh(Eigen::Index submesh_root) const { return aux.get_connected_submeshes(*data)[connected_submesh_index(submesh_root)]; }

	void traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
	void traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
};

