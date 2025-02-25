#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>

#include "defects/Defects.h"

class GeoRepair;

struct CallbackWidget : public igl::opengl::glfw::imgui::ImGuiWidget
{
	GeoRepair* geo_repair = nullptr;

	virtual bool key_down(int key, int modifiers) override;
};

class GeoRepair
{
	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin plugin;
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	CallbackWidget callback;
	Mesh mesh;
	DefectList defect_list;

public:
	void run();
	void handle_path_drop(int count, const char** paths);
	bool handle_key_down(int key, int modifiers);

private:
	void init();
	void load_mesh(const char* filename);
	void load_mesh();
	void save_mesh(const char* filename);
	void save_mesh();
	void render_gui();
	void refresh_mesh();

	void undo();
	void redo();
};

int main()
{
	GeoRepair().run();
}

bool CallbackWidget::key_down(int key, int modifiers)
{
	return geo_repair->handle_key_down(key, modifiers);
}

void GeoRepair::run()
{
	init();
	
	load_mesh("../assets/noise.obj");
	
	viewer.launch(false, "GeoRepair", 1920, 1080);
}

void GeoRepair::handle_path_drop(int count, const char** paths)
{
	if (count > 0)
		load_mesh(paths[0]);
}

bool GeoRepair::handle_key_down(int key, int modifiers)
{
	switch (key)
	{
	case GLFW_KEY_Z:
		if (modifiers & GLFW_MOD_CONTROL)
		{
			if (modifiers & GLFW_MOD_SHIFT)
				redo();
			else
				undo();
			return true;
		}
		return false;
	case GLFW_KEY_O:
		if (modifiers & GLFW_MOD_CONTROL)
		{
			load_mesh();
			return true;
		}
		return false;
	case GLFW_KEY_S:
		if (modifiers & GLFW_MOD_CONTROL)
		{
			save_mesh();
			return true;
		}
		return false;
	}
	return false;
}

static void path_drop_callback(GLFWwindow* window, int count, const char** paths)
{
	((GeoRepair*)glfwGetWindowUserPointer(window))->handle_path_drop(count, paths);
}

void GeoRepair::init()
{
	viewer.plugins.push_back(&plugin);
	callback.geo_repair = this;
	plugin.widgets.push_back(&callback);
	plugin.widgets.push_back(&menu);

	menu.callback_draw_viewer_window = [this]() { render_gui(); };
	viewer.callback_init = [this](decltype(viewer)& viewer)
		{
			glfwSetWindowUserPointer(viewer.window, this);
			glfwSetDropCallback(viewer.window, &path_drop_callback);
			return false;
		};
	viewer.data().point_size = 5.0f;
}

void GeoRepair::load_mesh(const char* filename)
{
	if (mesh.load(filename))
		refresh_mesh();
}

void GeoRepair::load_mesh()
{
	load_mesh(igl::file_dialog_open().c_str());
}

void GeoRepair::save_mesh(const char* filename)
{
	mesh.save(filename);
}

void GeoRepair::save_mesh()
{
	save_mesh(igl::file_dialog_save().c_str());
}

void GeoRepair::render_gui()
{
	ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Geo Repair", nullptr, ImGuiWindowFlags_NoSavedSettings))
	{
		if (ImGui::Button("Load Mesh"))
			load_mesh();
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("Ctrl+O");
		ImGui::SameLine();
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("Ctrl+S");
		if (ImGui::Button("Save Mesh"))
			save_mesh();
		if (ImGui::Button("Undo"))
			undo();
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("Ctrl+Z");
		ImGui::SameLine();
		if (ImGui::Button("Redo"))
			redo();
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("Ctrl+Shift+Z");

		if (ImGui::CollapsingHeader("Noise Smoothing"))
		{
			static const char* detection_methods[] = { "Laplacian Residual", "Mean Curvature", "Feature Sensitive", "Eigen Values" };
			auto& noise_smoothing = defect_list.get<Defect::NOISE_SMOOTHING>();
			if (ImGui::BeginCombo("Detection Method", detection_methods[(int)noise_smoothing.detection_method]))
			{
				for (int i = 0; i < 4; ++i)
				{
					if (ImGui::Selectable(detection_methods[i], i == (int)noise_smoothing.detection_method))
						noise_smoothing.detection_method = decltype(noise_smoothing.detection_method)(i);
					if (i == (int)noise_smoothing.detection_method)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}
			static const char* smoothing_methods[] = { "Laplacian", "Taubin", "Desbrun", "Bilateral" };
			if (ImGui::BeginCombo("Smoothing Method", smoothing_methods[(int)noise_smoothing.smoothing_method]))
			{
				for (int i = 0; i < 4; ++i)
				{
					if (ImGui::Selectable(smoothing_methods[i], i == (int)noise_smoothing.smoothing_method))
						noise_smoothing.smoothing_method = decltype(noise_smoothing.smoothing_method)(i);
					if (i == (int)noise_smoothing.smoothing_method)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}
			// TODO noise smoothing parameters
			ImGui::BeginDisabled(noise_smoothing.in_detected_state()); // TODO later, in_detected_state() will need to check for the particular selection of the mesh
			if (ImGui::Button("Detect"))
			{
				noise_smoothing.detect(mesh);
				if (noise_smoothing.in_detected_state())
				{
					// TODO highlight noisy vertices
				}
				else
				{
					// TODO message that nothing was detected
				}
			}
			ImGui::SameLine();
			ImGui::EndDisabled();
			ImGui::BeginDisabled(!noise_smoothing.in_detected_state());
			if (ImGui::Button("Repair"))
			{
				noise_smoothing.repair(mesh);
				refresh_mesh();
			}
			ImGui::SameLine();
			if (ImGui::Button("Reset"))
				noise_smoothing.reset();
			ImGui::EndDisabled();
		}
		ImGui::End();
	}
}

void GeoRepair::refresh_mesh()
{
	auto& data = viewer.data();
	data.clear();
	data.set_mesh(mesh.get_vertices(), mesh.get_faces());
	Eigen::MatrixXd vertex_colors(1, 3);
	(vertex_colors << 0.4, 0.4, 0.4).finished();
	data.set_points(mesh.get_vertices(), vertex_colors);
	data.set_face_based(true);
	defect_list.reset_all();
}

void GeoRepair::undo()
{
	if (mesh.undo())
		refresh_mesh();
}

void GeoRepair::redo()
{
	if (mesh.redo())
		refresh_mesh();
}
