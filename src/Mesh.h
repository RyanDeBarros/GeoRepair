#pragma once

#include "MeshHistory.h"

class Mesh
{
	MeshHistory history;
	std::shared_ptr<MeshData> data;

public:
	Mesh() : data(std::make_shared<MeshData>()) {}

	bool load(const char* filename);
	bool save(const char* filename);

	void push();
	bool undo();
	bool redo();

	int history_index() const { return history.index(); }
	const std::string& get_filename() const { return data->primary.mesh_filename; }

	const decltype(data->primary.V)& get_vertices() const { return data->primary.V; }
	decltype(data->primary.V)& get_vertices() { return data->primary.V; }
	const decltype(data->primary.F)& get_faces() const { return data->primary.F; }
	decltype(data->primary.F)& get_faces() { return data->primary.F; }
	const decltype(data->primary.VC)& get_vertex_colors() const { return data->primary.VC; }
	decltype(data->primary.VC)& get_vertex_colors() { return data->primary.VC; }
	const decltype(data->primary.FC)& get_face_colors() const { return data->primary.FC; }
	decltype(data->primary.FC)& get_face_colors() { return data->primary.FC; }
	const decltype(data->primary.E1)& get_edges_1() const { return data->primary.E1; }
	decltype(data->primary.E1)& get_edges_1() { return data->primary.E1; }
	const decltype(data->primary.E1)& get_edges_2() const { return data->primary.E2; }
	decltype(data->primary.E1)& get_edges_2() { return data->primary.E2; }
	const decltype(data->primary.EC)& get_edge_colors() const { return data->primary.EC; }
	decltype(data->primary.EC)& get_edge_colors() { return data->primary.EC; }

	void reset_vertex_colors();
	void reset_face_colors();
	void reset_edge_colors();

	const std::vector<Eigen::Index>& get_adj_vertices(Eigen::Index vertex_index) const { return data->get_vadj()[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_rows(Eigen::Index vertex_index) const { return data->get_vfadj_row()[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_cols(Eigen::Index vertex_index) const { return data->get_vfadj_col()[vertex_index]; }
	Eigen::SparseMatrix<int>::InnerIterator get_adj_face_iterator(Eigen::Index face_index) const { return Eigen::SparseMatrix<int>::InnerIterator(data->get_fadj(), face_index); }
	const std::vector<std::vector<Eigen::Index>> get_boundary_loops() const { return data->get_boundary_loops(); }
	const std::unordered_set<Eigen::Index> get_boundary_vertices() const { return data->get_boundary_vertices(); }
	const Eigen::MatrixXd& get_vertex_normals() const { return data->get_vertex_normals(); }
	const Eigen::SparseMatrix<double>& get_laplacian() const { return data->get_laplacian(); }
	Eigen::SparseMatrix<double>::InnerIterator get_laplacian_iterator(Eigen::Index vertex_index) const { return Eigen::SparseMatrix<double>::InnerIterator(data->get_laplacian(), vertex_index); }
	const Eigen::MatrixXd& get_laplacian_eval() const { return data->get_laplacian_eval(); }
	const Eigen::VectorXd& get_laplacian_residuals() const { return data->get_laplacian_residuals(); }
	Eigen::SparseMatrix<double>::InnerIterator get_mass_iterator(Eigen::Index vertex_index) const { return Eigen::SparseMatrix<double>::InnerIterator(data->get_mass(), vertex_index); }
	const Eigen::MatrixXd& get_mean_curvatures() const { return data->get_mean_curvatures(); }
	const Eigen::VectorXd& get_mean_curvature_magnitudes() const { return data->get_mean_curvature_magnitudes(); }

	const std::vector<std::vector<Eigen::Index>>& get_connected_submeshes() const { return data->get_connected_submeshes(); }
	const std::unordered_map<Eigen::Index, Eigen::Index>& get_connected_submesh_roots() const { return data->get_connected_submesh_roots(); }
	size_t connected_submesh_index(Eigen::Index submesh_root) const { return data->get_connected_submesh_roots().find(submesh_root)->second; }
	const std::vector<Eigen::Index>& get_connected_submesh(Eigen::Index submesh_root) const { return data->get_connected_submeshes()[connected_submesh_index(submesh_root)]; }

	void traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
	void traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const;
};

