#include <igl/opengl/glfw/Viewer.h>

int main()
{
	igl::opengl::glfw::Viewer viewer;
	viewer.launch(false, "GeoRepair", 1920, 1080);
}
