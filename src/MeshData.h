#pragma once

#include <igl/opengl/ViewerData.h>
#include <igl/AABB.h>

class MeshData
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	std::vector<std::vector<Eigen::Index>> VADJ;
	std::vector<std::vector<Eigen::Index>> FADJ_row;
	std::vector<std::vector<Eigen::Index>> FADJ_col;
	igl::AABB<Eigen::MatrixXd, 3> tree;

public:
	bool load(const char* filename);
	bool save(const char* filename);
	void refresh();

	const decltype(V)& get_vertices() const { return V; }
	decltype(V)& get_vertices() { return V; }
	const decltype(F)& get_faces() const { return F; }
	decltype(F)& get_faces() { return F; }
	const std::vector<Eigen::Index>& get_adj_vertices(Eigen::Index vertex_index) const { return VADJ[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_rows(Eigen::Index vertex_index) const { return FADJ_row[vertex_index]; }
	const std::vector<Eigen::Index>& get_adj_face_cols(Eigen::Index vertex_index) const { return FADJ_col[vertex_index]; }
	const decltype(tree)& get_tree() const { return tree; }

	// TODO store cached structures used by defect algorithms?
};
