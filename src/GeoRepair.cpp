#include <igl/opengl/glfw/Viewer.h>

#include "MeshData.h"
#include "defects/DuplicateFaces.h"
#include "defects/NullFaces.h"

int main()
{
	igl::opengl::glfw::Viewer viewer;
	MeshData mesh;
	//mesh.load("../assets/duplicate face.obj");
	mesh.load("../assets/null face.obj");

	//defects::DuplicateFaces duplicate_faces;
	//duplicate_faces.detect(mesh);
	//duplicate_faces.repair(mesh);
	defects::NullFaces null_faces;
	null_faces.detect(mesh);
	null_faces.repair(mesh);
	//defects::DegenerateFaces degenerate_faces;
	//degenerate_faces.acceptance = 0.1;
	//degenerate_faces.detect(mesh);
	//degenerate_faces.repair(mesh);
	mesh.refresh();

	//mesh.load("../assets/duplicate face - repaired.obj");
	mesh.save("../assets/null face - repaired.obj");

	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
	viewer.data().point_size = 5.0f;
	viewer.data().face_based = true;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
