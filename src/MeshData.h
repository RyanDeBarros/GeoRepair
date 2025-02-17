#pragma once

#include <igl/opengl/ViewerData.h>
#include <igl/AABB.h>

class MeshData
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	std::vector<std::vector<int>> ADJ;
	igl::AABB<Eigen::MatrixXd, 3> tree;

public:
	bool load(const char* filename);
	bool save(const char* filename);

	const decltype(V)& get_vertices() const { return V; }
	const decltype(F)& get_faces() const { return F; }
};
