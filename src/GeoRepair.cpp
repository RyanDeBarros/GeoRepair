#include <igl/opengl/glfw/Viewer.h>

#include "defects/UnboundVertices.h"

int main()
{
	igl::opengl::glfw::Viewer viewer;
	Mesh mesh;
	assert(mesh.load("../assets/invalid values.obj"));

	defects::UnboundVertices unbound_vertices;
	unbound_vertices.min_x = -10;
	unbound_vertices.max_x = 10;
	unbound_vertices.min_y = -10;
	unbound_vertices.max_y = 10;
	unbound_vertices.min_z = -10;
	unbound_vertices.max_z = 10;
	unbound_vertices.detect(mesh);
	unbound_vertices.repair(mesh);
	
	mesh.undo();
	mesh.redo();

	assert(mesh.save("../assets/invalid values - repaired.obj"));

	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
	viewer.data().point_size = 5.0f;
	viewer.data().face_based = true;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
