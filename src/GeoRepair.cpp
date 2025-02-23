#include <igl/opengl/glfw/Viewer.h>

#include "defects/UnpatchedHoles.h"
//#include "defects/InvertedNormals.h"

int main()
{
	igl::opengl::glfw::Viewer viewer;
	MeshData mesh;
	assert(mesh.load("../assets/hole.obj"));

	defects::UnpatchedHoles unpatched_holes;
	//unpatched_holes.patch_method = defects::UnpatchedHoles::PatchMethod::FAN;
	//unpatched_holes.patch_method = defects::UnpatchedHoles::PatchMethod::STRIP;
	unpatched_holes.patch_method = defects::UnpatchedHoles::PatchMethod::CLIP;
	//unpatched_holes.patch_method = defects::UnpatchedHoles::PatchMethod::PIE;
	unpatched_holes.detect(mesh);
	unpatched_holes.repair(mesh);
	mesh.refresh_data_structures();

	//defects::InvertedNormals inverted_normals;
	//inverted_normals.detect(mesh);
	//inverted_normals.repair(mesh);
	//mesh.refresh_data_structures();

	//assert(mesh.save("../assets/hole - fan.obj"));
	//assert(mesh.save("../assets/hole - strip.obj"));
	assert(mesh.save("../assets/hole - clip.obj"));
	//assert(mesh.save("../assets/hole - pie.obj"));

	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
	viewer.data().point_size = 5.0f;
	viewer.data().face_based = true;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
