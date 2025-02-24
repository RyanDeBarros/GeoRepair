#include <igl/opengl/glfw/Viewer.h>

#include "defects/NoiseSmoothing.h"

int main()
{
	igl::opengl::glfw::Viewer viewer;
	Mesh mesh;
	assert(mesh.load("../assets/noise.obj"));

	defects::NoiseSmoothing noise_smoothing;
	noise_smoothing.detection_method = defects::NoiseSmoothing::DetectionMethod::LAPLACIAN_RESIDUAL;
	noise_smoothing.smoothing_method = defects::NoiseSmoothing::SmoothingMethod::LAPLACIAN;
	noise_smoothing.detect(mesh);
	noise_smoothing.repair(mesh);

	assert(mesh.save("../assets/noise - laplacian smoothing - laplacian residual.obj"));

	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
	viewer.data().point_size = 5.0f;
	viewer.data().face_based = true;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
