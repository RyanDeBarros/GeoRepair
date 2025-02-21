#include <igl/opengl/glfw/Viewer.h>

#include "MeshData.h"
#include "defects/DuplicateFaces.h"
#include "defects/NullFaces.h"
#include "defects/DegenerateVertexPatch.h"
#include "defects/DegenerateFaces.h"
#include "defects/InvertedNormals.h"

// TODO throughout project, use vector<bool> instead of unordered_set<Eigen::Index> for vertex/face index checking

int main()
{
	igl::opengl::glfw::Viewer viewer;
	MeshData mesh;
	//mesh.load("../assets/tetrahedron - inverted normals.obj");
	mesh.load("../assets/inverted normals.obj");

	defects::InvertedNormals inverted_normals;
	inverted_normals.detect(mesh);
	inverted_normals.repair(mesh);
	//inverted_normals.flip(mesh);
	mesh.refresh_data_structures();

	//mesh.save("../assets/tetrahedron - inverted normals - repaired.obj");
	mesh.save("../assets/inverted normals - repaired.obj");

	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
	viewer.data().point_size = 5.0f;
	viewer.data().face_based = true;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
