#include <igl/opengl/glfw/Viewer.h>

#include "defects/NonManifoldEdges.h"

int main()
{
	igl::opengl::glfw::Viewer viewer;
	MeshData mesh;
	assert(mesh.load("../assets/non-manifold.obj"));
	defects::NonManifoldEdges non_manifold_edges;
	non_manifold_edges.detect(mesh);
	//non_manifold_edges.repair(mesh);
	//mesh.refresh_data_structures();

	non_manifold_edges.traverse_non_manifold_edges([](Eigen::Index v1, Eigen::Index v2, size_t count) {
		std::cout << "(" << v1 << ", " << v2 << "): " << count << std::endl;
		});

	assert(mesh.save("../assets/non-manifold.obj"));

	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
	viewer.data().point_size = 5.0f;
	viewer.data().face_based = true;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
