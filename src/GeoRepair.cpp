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

	// TODO option to detect all and repair all in menu

	int opened_header = -1;
	void render_menu_gui();
	void render_degenerate_faces_gui();
	void render_degenerate_vertex_patch_gui();
	void render_duplicate_faces_gui();
	void render_general_duplicate_vertices_gui();
	void render_inverted_normals_gui();
	void render_noise_smoothing_gui();
	void render_non_manifold_edges_gui();
	void render_null_faces_gui();
	void render_unbound_vertices_gui();
	void render_unpatched_holes_gui();
	
	void render_defect_base_buttons(Defect defect, void(GeoRepair::*detect_success)());
	bool no_detection = false;
	void message_no_detection();

	void degenerate_faces_detect_success();
	void degenerate_vertex_patch_detect_success();
	void duplicate_faces_detect_success();
	void general_duplicate_vertices_detect_success();
	void inverted_normals_detect_success();
	void noise_smoothing_detect_success();
	void non_manifold_edges_detect_success();
	void null_faces_detect_success();
	void unbound_vertices_detect_success();
	void unpatched_holes_detect_success();
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
		render_menu_gui();
		render_degenerate_faces_gui();
		render_degenerate_vertex_patch_gui();
		render_duplicate_faces_gui();
		render_general_duplicate_vertices_gui();
		render_inverted_normals_gui();
		render_noise_smoothing_gui();
		render_non_manifold_edges_gui();
		render_null_faces_gui();
		render_unbound_vertices_gui();
		render_unpatched_holes_gui();
		message_no_detection();
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

void GeoRepair::render_menu_gui()
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
}

void GeoRepair::render_degenerate_faces_gui()
{
	if (opened_header != -1 && opened_header != Defect::DEGENERATE_FACES)
		ImGui::SetNextItemOpen(false);
	if (ImGui::CollapsingHeader("Degenerate Faces"))
	{
		opened_header = Defect::DEGENERATE_FACES;
		auto& degenerate_faces = defect_list.get<Defect::DEGENERATE_FACES>();
		bool reset = false;

		double tolerance = degenerate_faces.tolerance;
		if (ImGui::InputDouble("Tolerance", &tolerance))
		{
			tolerance = std::max(tolerance, 0.0);
			if (tolerance != degenerate_faces.tolerance)
			{
				degenerate_faces.tolerance = tolerance;
				reset = true;
			}
		}

		if (ImGui::Checkbox("Ignore normals", &degenerate_faces.ignore_normals))
			reset = true;

		if (reset)
			degenerate_faces.reset();

		render_defect_base_buttons(Defect::DEGENERATE_FACES, &GeoRepair::degenerate_faces_detect_success);
	}
	else if (opened_header == Defect::DEGENERATE_FACES)
		opened_header = -1;
}

void GeoRepair::render_degenerate_vertex_patch_gui()
{
	// TODO
}

void GeoRepair::render_duplicate_faces_gui()
{
	// TODO
}

void GeoRepair::render_general_duplicate_vertices_gui()
{
	// TODO
}

void GeoRepair::render_inverted_normals_gui()
{
	// TODO
}

void GeoRepair::render_noise_smoothing_gui()
{
	if (opened_header != -1 && opened_header != Defect::NOISE_SMOOTHING)
		ImGui::SetNextItemOpen(false);
	if (ImGui::CollapsingHeader("Noise Smoothing"))
	{
		opened_header = Defect::NOISE_SMOOTHING;
		auto& noise_smoothing = defect_list.get<Defect::NOISE_SMOOTHING>();
		bool reset = false;
		int prev_combo = -1;

		static const char* detection_methods[] = { "Laplacian Residual", "Mean Curvature", "Feature Sensitive", "Eigen Values" };
		prev_combo = (int)noise_smoothing.detection_method;
		ImGui::SetNextItemWidth(200);
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
		if ((int)noise_smoothing.detection_method != prev_combo)
			reset = true;

		static const char* smoothing_methods[] = { "Laplacian", "Taubin", "Desbrun", "Bilateral" };
		prev_combo = (int)noise_smoothing.smoothing_method;
		ImGui::SetNextItemWidth(200);
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
		if ((int)noise_smoothing.smoothing_method != prev_combo)
			reset = true;

		// TODO noise smoothing parameters

		if (reset)
			noise_smoothing.reset();

		render_defect_base_buttons(Defect::NOISE_SMOOTHING, &GeoRepair::noise_smoothing_detect_success);
	}
	else if (opened_header == Defect::NOISE_SMOOTHING)
		opened_header = -1;
}

void GeoRepair::render_non_manifold_edges_gui()
{
	// TODO
}

void GeoRepair::render_null_faces_gui()
{
	// TODO
}

void GeoRepair::render_unbound_vertices_gui()
{
	// TODO
}

void GeoRepair::render_unpatched_holes_gui()
{
	// TODO
}

void GeoRepair::render_defect_base_buttons(Defect defect, void(GeoRepair::*detect_success)())
{
	ImGui::BeginDisabled(defect_list[defect].in_detected_state()); // TODO later, in_detected_state() will need to check for the particular selection of the mesh
	if (ImGui::Button("Detect"))
	{
		defect_list[defect].detect(mesh);
		if (defect_list[defect].in_detected_state())
			(this->*detect_success)();
		else
			no_detection = true;
	}
	ImGui::SameLine();
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!defect_list[defect].in_detected_state());
	if (ImGui::Button("Repair"))
	{
		defect_list[defect].repair(mesh);
		refresh_mesh();
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset"))
		defect_list[defect].reset();
	ImGui::EndDisabled();
}

void GeoRepair::message_no_detection()
{
	if (no_detection)
	{
		ImGui::OpenPopup("Info##nodetection");
		no_detection = false;
	}
	if (ImGui::BeginPopupModal("Info##nodetection", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::Text("No defects were detected!");
		ImGui::Separator();
		if (ImGui::Button("OK"))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
}

void GeoRepair::degenerate_faces_detect_success()
{
	// TODO
}

void GeoRepair::degenerate_vertex_patch_detect_success()
{
	// TODO
}

void GeoRepair::duplicate_faces_detect_success()
{
	// TODO
}

void GeoRepair::general_duplicate_vertices_detect_success()
{
	// TODO
}

void GeoRepair::inverted_normals_detect_success()
{
	// TODO
}

void GeoRepair::noise_smoothing_detect_success()
{
	// TODO
}

void GeoRepair::non_manifold_edges_detect_success()
{
	// TODO
}

void GeoRepair::null_faces_detect_success()
{
	// TODO
}

void GeoRepair::unbound_vertices_detect_success()
{
	// TODO
}

void GeoRepair::unpatched_holes_detect_success()
{
	// TODO
}
