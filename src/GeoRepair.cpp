#define IGL_VIEWER_VIEWER_QUIET

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>

#include "defects/Defects.h"

// TODO later add settings for these colors and point/edge sizes
namespace colors
{
	namespace style
	{
		ImVec4 detected = ImVec4(0.3f, 0.65f, 0.35f, 1.0f);
	}
	namespace vertex
	{
		Eigen::RowVector3d neutral = Eigen::RowVector3d(0.5, 0.5, 0.5);
		Eigen::RowVector3d defective = Eigen::RowVector3d(0.9, 0.3, 0.3);
		Eigen::RowVector3d defective_low = Eigen::RowVector3d(0.7, 0.3, 0.6);
	}
	namespace face
	{
		Eigen::RowVector3d neutral = Eigen::RowVector3d(1.0, 0.894, 0.235);
		Eigen::RowVector3d defective = Eigen::RowVector3d(1.0, 0.0, 0.784);
		Eigen::RowVector3d defective_alt = Eigen::RowVector3d(0.392, 1.0, 0.392);
	}
	namespace edge
	{
		Eigen::RowVector3d defective = Eigen::RowVector3d(0.0, 1.0, 0.0);
		Eigen::RowVector3d defective_alt = Eigen::RowVector3d(0.0, 0.7, 0.7);
	}
}

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

	GLFWcursor* wait_cursor = nullptr;
	GLFWcursor* arrow_cursor = nullptr;

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
	igl::opengl::ViewerData& mesh_data() { return viewer.data(0); }
	igl::opengl::ViewerData& edge_data() { return viewer.data(1); }
	void render_gui();
	void refresh_mesh();

	void undo();
	void redo();
	void clear_visualisation();
	void restore_visualisation();

	int opened_header = -1;
	void render_menu_gui();
	void render_header_gui();
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

	void reset_colors();
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
	case GLFW_KEY_C:
		if (modifiers & GLFW_MOD_SHIFT)
			restore_visualisation();
		else
			clear_visualisation();
		return true;
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
			wait_cursor = glfwCreateStandardCursor(GLFW_NOT_ALLOWED_CURSOR);
			arrow_cursor = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);


			// TODO remove
			load_mesh("../assets/noise.obj");



			return false;
		};
	mesh_data().point_size = 8.0f;

	viewer.append_mesh(); // append edge mesh
	edge_data().show_faces = false;
	edge_data().line_width = 7.5f;
}

void GeoRepair::load_mesh(const char* filename)
{
	if (mesh.load(filename))
	{
		refresh_mesh();
		reset_colors();
	}
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
	ImGui::SetNextWindowSize(ImVec2(400, 600), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Geo Repair", nullptr, ImGuiWindowFlags_NoSavedSettings))
	{
		render_menu_gui();
		render_header_gui();
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
	mesh_data().clear();
	mesh_data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	mesh_data().set_points(mesh.get_vertices(), mesh.get_vertex_colors());
	mesh_data().set_face_based(true);
	restore_visualisation();
	defect_list.reset_all();
	if (viewer.window)
	{
		std::string title = "GeoRepair - " + mesh.get_filename() + " - " + std::to_string(mesh.history_index());
		glfwSetWindowTitle(viewer.window, title.c_str());
	}
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

void GeoRepair::clear_visualisation()
{
	reset_colors();
}

void GeoRepair::restore_visualisation()
{
	mesh_data().set_points(mesh.get_vertices(), mesh.get_vertex_colors());
	mesh_data().set_colors(mesh.get_face_colors());
	edge_data().clear_edges();
	edge_data().add_edges(mesh.get_edges_1(), mesh.get_edges_2(), mesh.get_edge_colors());
}

void GeoRepair::render_menu_gui()
{
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("Load mesh", "Ctrl+O"))
				load_mesh();
			if (ImGui::MenuItem("Save mesh", "Ctrl+S"))
				save_mesh();
			if (ImGui::MenuItem("Info"))
				ImGui::OpenPopup("Info");
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Edit"))
		{
			if (ImGui::MenuItem("Undo", "Ctrl+Z"))
				undo();
			if (ImGui::MenuItem("Redo", "Ctrl+Shft+Z"))
				redo();
			ImGui::Separator();
			if (ImGui::MenuItem("Clear visualization", "C"))
				clear_visualisation();
			if (ImGui::MenuItem("Restore visualization", "Shft+C"))
				restore_visualisation();
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}

	if (ImGui::BeginPopupModal("Info", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::Text("GeoRepair detects and repairs common mesh defects.");
		ImGui::Text("Note that triangulation happens automatically, faceless edges are discarded, and no information about texture coordinates, materials, etc. will be retained."); // TODO if this is changed, update disclaimer
		if (ImGui::Button("OK"))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
}

void GeoRepair::render_header_gui()
{
	if (ImGui::Button("Detect All"))
	{
		reset_colors();
		glfwSetCursor(viewer.window, wait_cursor);
		bool any_detected = false;
		for (size_t i = 0; i < defect_list.size(); ++i)
		{
			defect_list[(Defect)i].detect(mesh);
			if (defect_list[(Defect)i].in_detected_state())
				any_detected = true;
		}
		glfwSetCursor(viewer.window, arrow_cursor);
		if (any_detected)
		{
			mesh.reset_vertex_colors();
			mesh.reset_face_colors();
			mesh.reset_edge_colors();
			restore_visualisation();
		}
		else
			no_detection = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Repair All"))
	{
		glfwSetCursor(viewer.window, wait_cursor);
		defect_list.repair_all(mesh);
		glfwSetCursor(viewer.window, arrow_cursor);
		refresh_mesh();
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset All"))
	{
		glfwSetCursor(viewer.window, wait_cursor);
		defect_list.reset_all();
		glfwSetCursor(viewer.window, arrow_cursor);
		reset_colors();
	}
}

#define render_defect_gui_header(defect)\
	bool detected = defect_list[defect].in_detected_state();\
	if (detected) ImGui::PushStyleColor(ImGuiCol_Header, colors::style::detected);\
	if (opened_header != -1 && opened_header != defect)\
		ImGui::SetNextItemOpen(false);
#define render_defect_gui_footer(defect)\
	else if (opened_header == defect)\
		opened_header = -1;\
	if (detected)\
		ImGui::PopStyleColor();

void GeoRepair::render_degenerate_faces_gui()
{
	render_defect_gui_header(Defect::DEGENERATE_FACES);
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
	render_defect_gui_footer(Defect::DEGENERATE_FACES);
}

void GeoRepair::render_degenerate_vertex_patch_gui()
{
	render_defect_gui_header(Defect::DEGENERATE_VERTEX_PATCH);
	if (ImGui::CollapsingHeader("Degenerate Vertex Patches"))
	{
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Note that repairing may cause duplicate faces. Use Degenerate Faces if that's not wanted.");
		opened_header = Defect::DEGENERATE_VERTEX_PATCH;
		auto& degenerate_vertex_patch = defect_list.get<Defect::DEGENERATE_VERTEX_PATCH>();
		bool reset = false;

		double tolerance = degenerate_vertex_patch.tolerance;
		if (ImGui::InputDouble("Tolerance", &tolerance))
		{
			tolerance = std::max(tolerance, 0.0);
			if (tolerance != degenerate_vertex_patch.tolerance)
			{
				degenerate_vertex_patch.tolerance = tolerance;
				reset = true;
			}
		}

		if (reset)
			degenerate_vertex_patch.reset();

		render_defect_base_buttons(Defect::DEGENERATE_VERTEX_PATCH, &GeoRepair::degenerate_vertex_patch_detect_success);
	}
	render_defect_gui_footer(Defect::DEGENERATE_VERTEX_PATCH);
}

void GeoRepair::render_duplicate_faces_gui()
{
	render_defect_gui_header(Defect::DUPLICATE_FACES);
	if (ImGui::CollapsingHeader("Duplicate Faces"))
	{
		opened_header = Defect::DUPLICATE_FACES;
		auto& duplicate_faces = defect_list.get<Defect::DUPLICATE_FACES>();
		bool reset = false;

		if (ImGui::Checkbox("Ignore normals", &duplicate_faces.ignore_normals))
			reset = true;

		if (reset)
			duplicate_faces.reset();

		render_defect_base_buttons(Defect::DUPLICATE_FACES, &GeoRepair::duplicate_faces_detect_success);
	}
	render_defect_gui_footer(Defect::DUPLICATE_FACES);
}

void GeoRepair::render_general_duplicate_vertices_gui()
{
	render_defect_gui_header(Defect::GENERAL_DUPLICATE_VERTICES);
	if (ImGui::CollapsingHeader("Duplicate Vertices"))
	{
		opened_header = Defect::GENERAL_DUPLICATE_VERTICES;
		auto& general_duplicate_vertices = defect_list.get<Defect::GENERAL_DUPLICATE_VERTICES>();
		bool reset = false;

		double tolerance = general_duplicate_vertices.tolerance;
		if (ImGui::InputDouble("Tolerance", &tolerance))
		{
			tolerance = std::max(tolerance, 0.0);
			if (tolerance != general_duplicate_vertices.tolerance)
			{
				general_duplicate_vertices.tolerance = tolerance;
				reset = true;
			}
		}

		if (reset)
			general_duplicate_vertices.reset();

		render_defect_base_buttons(Defect::GENERAL_DUPLICATE_VERTICES, &GeoRepair::general_duplicate_vertices_detect_success);
	}
	render_defect_gui_footer(Defect::GENERAL_DUPLICATE_VERTICES);
}

void GeoRepair::render_inverted_normals_gui()
{
	render_defect_gui_header(Defect::INVERTED_NORMALS);
	if (ImGui::CollapsingHeader("Inverted Normals"))
	{
		opened_header = Defect::INVERTED_NORMALS;
		auto& inverted_normals = defect_list.get<Defect::INVERTED_NORMALS>();

		if (ImGui::Button("Flip All"))
		{
			inverted_normals.flip(mesh);
			refresh_mesh();
		}

		render_defect_base_buttons(Defect::INVERTED_NORMALS, &GeoRepair::inverted_normals_detect_success);
	}
	render_defect_gui_footer(Defect::INVERTED_NORMALS);
}

void GeoRepair::render_noise_smoothing_gui()
{
	render_defect_gui_header(Defect::NOISE_SMOOTHING);
	if (ImGui::CollapsingHeader("Noise Smoothing"))
	{
		opened_header = Defect::NOISE_SMOOTHING;
		auto& noise_smoothing = defect_list.get<Defect::NOISE_SMOOTHING>();
		bool reset = false;
		int prev_combo = -1;

		static const int num_detection_methods = 3;
		static const char* detection_methods[num_detection_methods] = { "Laplacian Residual", "Mean Curvature", "Feature Sensitive" };
		prev_combo = (int)noise_smoothing.detection_method;
		ImGui::SetNextItemWidth(200);
		if (ImGui::BeginCombo("Detection Method", detection_methods[(int)noise_smoothing.detection_method]))
		{
			for (int i = 0; i < num_detection_methods; ++i)
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

		if (noise_smoothing.detection_method == decltype(noise_smoothing.detection_method)::LAPLACIAN_RESIDUAL
			|| noise_smoothing.detection_method == decltype(noise_smoothing.detection_method)::FEATURE_SENSITIVE)
		{
			double sensitivity = noise_smoothing.laplacian_sensitivity;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputDouble("Laplacian Sensitivity", &sensitivity) && sensitivity >= 0.0)
			{
				noise_smoothing.laplacian_sensitivity = sensitivity;
				reset = true;
			}
		}
		if (noise_smoothing.detection_method == decltype(noise_smoothing.detection_method)::MEAN_CURVATURE
			|| noise_smoothing.detection_method == decltype(noise_smoothing.detection_method)::FEATURE_SENSITIVE)
		{
			double sensitivity = noise_smoothing.curvature_sensitivity;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputDouble("Curvature Sensitivity", &sensitivity) && sensitivity >= 0.0)
			{
				noise_smoothing.curvature_sensitivity = sensitivity;
				reset = true;
			}
		}
		if (ImGui::Checkbox("Smooth All", &noise_smoothing.smooth_all))
			reset = true;
		if (ImGui::Checkbox("Ignore Boundaries", &noise_smoothing.ignore_boundaries))
			reset = true;

		ImGui::Separator();

		static const int num_smoothing_methods = 4;
		static const char* smoothing_methods[num_smoothing_methods] = { "Laplacian", "Taubin", "Desbrun", "Bilateral" };
		prev_combo = (int)noise_smoothing.smoothing_method;
		ImGui::SetNextItemWidth(200);
		if (ImGui::BeginCombo("Smoothing Method", smoothing_methods[(int)noise_smoothing.smoothing_method]))
		{
			for (int i = 0; i < num_smoothing_methods; ++i)
			{
				if (ImGui::Selectable(smoothing_methods[i], i == (int)noise_smoothing.smoothing_method))
					noise_smoothing.smoothing_method = decltype(noise_smoothing.smoothing_method)(i);
				if (i == (int)noise_smoothing.smoothing_method)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		switch (noise_smoothing.smoothing_method)
		{
		case decltype(noise_smoothing.smoothing_method)::LAPLACIAN:
		{
			int iterations = noise_smoothing.laplacian_iterations;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputInt("Iterations", &iterations) && iterations >= 1)
				noise_smoothing.laplacian_iterations = iterations;
			ImGui::SetNextItemWidth(200);
			ImGui::SliderFloat("Smoothing Factor", &noise_smoothing.laplacian_smoothing_factor, 0.0f, 1.0f);
			break;
		}
		case decltype(noise_smoothing.smoothing_method)::TAUBIN:
		{
			int iterations = noise_smoothing.taubin_iterations;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputInt("Iterations", &iterations) && iterations >= 1)
				noise_smoothing.taubin_iterations = iterations;
			ImGui::SetNextItemWidth(200);
			ImGui::SliderFloat("Shrinking Factor", &noise_smoothing.taubin_shrinking_factor, 0.0f, 1.0f);
			ImGui::SetNextItemWidth(200);
			ImGui::SliderFloat("Expanding Factor", &noise_smoothing.taubin_expanding_factor, -1.0f, 0.0f);
			break;
		}
		case decltype(noise_smoothing.smoothing_method)::DESBRUN:
		{
			int iterations = noise_smoothing.desbrun_iterations;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputInt("Iterations", &iterations) && iterations >= 1)
				noise_smoothing.desbrun_iterations = iterations;
			ImGui::SetNextItemWidth(200);
			ImGui::SliderFloat("Smoothing Factor", &noise_smoothing.desbrun_smoothing_factor, 0.0f, 1.0f);
			break;
		}
		case decltype(noise_smoothing.smoothing_method)::BILATERAL:
		{
			int iterations = noise_smoothing.bilateral_iterations;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputInt("Iterations", &iterations) && iterations >= 1)
				noise_smoothing.bilateral_iterations = iterations;
			float factor = noise_smoothing.bilateral_tangential_factor;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputFloat("Tangential Factor", &factor) && factor > 0.0f)
				noise_smoothing.bilateral_tangential_factor = factor;
			factor = noise_smoothing.bilateral_normal_factor;
			ImGui::SetNextItemWidth(200);
			if (ImGui::InputFloat("Normal Factor", &factor) && factor > 0.0f)
				noise_smoothing.bilateral_normal_factor = factor;
			ImGui::SetNextItemWidth(200);
			ImGui::SliderFloat("Smoothing Factor", &noise_smoothing.bilateral_smoothing_factor, 0.0f, 1.0f);
			break;
		}
		}

		if (reset)
			noise_smoothing.reset();

		render_defect_base_buttons(Defect::NOISE_SMOOTHING, &GeoRepair::noise_smoothing_detect_success);
	}
	render_defect_gui_footer(Defect::NOISE_SMOOTHING);
}

void GeoRepair::render_non_manifold_edges_gui()
{
	render_defect_gui_header(Defect::NON_MANIFOLD_EDGES);
	if (ImGui::CollapsingHeader("Non-Manifold Edges"))
	{
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("No specific repair function supported for non-manifold edges.");
		opened_header = Defect::NON_MANIFOLD_EDGES;
		auto& non_manifold_edges = defect_list.get<Defect::NON_MANIFOLD_EDGES>();

		ImGui::BeginDisabled(non_manifold_edges.in_detected_state());
		if (ImGui::Button("Detect"))
		{
			reset_colors();
			glfwSetCursor(viewer.window, wait_cursor);
			non_manifold_edges.detect(mesh);
			glfwSetCursor(viewer.window, arrow_cursor);
			if (non_manifold_edges.in_detected_state())
				non_manifold_edges_detect_success();
			else
				no_detection = true;
		}
		ImGui::SameLine();
		ImGui::EndDisabled();
		ImGui::BeginDisabled(!non_manifold_edges.in_detected_state());
		if (ImGui::Button("Reset"))
		{
			glfwSetCursor(viewer.window, wait_cursor);
			non_manifold_edges.reset();
			glfwSetCursor(viewer.window, arrow_cursor);
			reset_colors();
		}
		ImGui::EndDisabled();
	}
	render_defect_gui_footer(Defect::NON_MANIFOLD_EDGES);
}

void GeoRepair::render_null_faces_gui()
{
	render_defect_gui_header(Defect::NULL_FACES);
	if (ImGui::CollapsingHeader("Null Faces"))
	{
		opened_header = Defect::NULL_FACES;
		auto& null_faces = defect_list.get<Defect::NULL_FACES>();

		render_defect_base_buttons(Defect::NULL_FACES, &GeoRepair::null_faces_detect_success);
	}
	render_defect_gui_footer(Defect::NULL_FACES);
}

void GeoRepair::render_unbound_vertices_gui()
{
	render_defect_gui_header(Defect::UNBOUND_VERTICES);
	if (ImGui::CollapsingHeader("Unbound Vertices"))
	{
		opened_header = Defect::UNBOUND_VERTICES;
		auto& unbound_vertices = defect_list.get<Defect::UNBOUND_VERTICES>();
		bool reset = false;
		
		float x[2] = { (float)unbound_vertices.min_x, (float)unbound_vertices.max_x };
		if (ImGui::InputFloat2("Min/Max X", x) && x[0] <= x[1])
		{
			unbound_vertices.min_x = x[0];
			unbound_vertices.max_x = x[1];
			reset = true;
		}
		float y[2] = { (float)unbound_vertices.min_y, (float)unbound_vertices.max_y };
		if (ImGui::InputFloat2("Min/Max Y", y) && y[0] <= y[1])
		{
			unbound_vertices.min_y = y[0];
			unbound_vertices.max_y = y[1];
			reset = true;
		}
		float z[2] = { (float)unbound_vertices.min_z, (float)unbound_vertices.max_z };
		if (ImGui::InputFloat2("Min/Max Z", z) && z[0] <= z[1])
		{
			unbound_vertices.min_z = z[0];
			unbound_vertices.max_z = z[1];
			reset = true;
		}

		if (reset)
			unbound_vertices.reset();

		render_defect_base_buttons(Defect::UNBOUND_VERTICES, &GeoRepair::unbound_vertices_detect_success);
	}
	render_defect_gui_footer(Defect::UNBOUND_VERTICES);
}

void GeoRepair::render_unpatched_holes_gui()
{
	render_defect_gui_header(Defect::UNPATCHED_HOLES);
	if (ImGui::CollapsingHeader("Unpatched Holes"))
	{
		opened_header = Defect::UNPATCHED_HOLES;
		auto& unpatched_holes = defect_list.get<Defect::UNPATCHED_HOLES>();

		static const char* patch_methods[] = { "Fan", "Strip", "Clip", "Pie" };
		ImGui::SetNextItemWidth(200);
		if (ImGui::BeginCombo("Patch Method", patch_methods[(int)unpatched_holes.patch_method]))
		{
			for (int i = 0; i < 4; ++i)
			{
				if (ImGui::Selectable(patch_methods[i], i == (int)unpatched_holes.patch_method))
					unpatched_holes.patch_method = decltype(unpatched_holes.patch_method)(i);
				if (i == (int)unpatched_holes.patch_method)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		render_defect_base_buttons(Defect::UNPATCHED_HOLES, &GeoRepair::unpatched_holes_detect_success);
	}
	render_defect_gui_footer(Defect::UNPATCHED_HOLES);
}

void GeoRepair::render_defect_base_buttons(Defect defect, void(GeoRepair::*detect_success)())
{
	ImGui::BeginDisabled(defect_list[defect].in_detected_state()); // TODO later, in_detected_state() will need to check for the particular selection of the mesh
	if (ImGui::Button("Detect"))
	{
		reset_colors();
		glfwSetCursor(viewer.window, wait_cursor);
		defect_list[defect].detect(mesh);
		glfwSetCursor(viewer.window, arrow_cursor);
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
		glfwSetCursor(viewer.window, wait_cursor);
		defect_list[defect].repair(mesh);
		glfwSetCursor(viewer.window, arrow_cursor);
		refresh_mesh();
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset"))
	{
		glfwSetCursor(viewer.window, wait_cursor);
		defect_list[defect].reset();
		glfwSetCursor(viewer.window, arrow_cursor);
		reset_colors();
	}
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

static void detect_success_for_degenerate_vertex_patch(igl::opengl::ViewerData& viewer_data, Mesh& mesh,
	const defects::DegenerateVertexPatch& degenerate_vertex_patch, const Eigen::RowVector3d& neutral_color, const Eigen::RowVector3d& defective_color)
{
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resizeLike(mesh.get_vertices());
	auto& vertex_clusters = degenerate_vertex_patch.get_vertex_clusters();
	for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
	{
		if (vertex_clusters.exists(i))
			vertex_colors.row(i) = defective_color;
		else
			vertex_colors.row(i) = neutral_color;
	}
	viewer_data.set_points(mesh.get_vertices(), vertex_colors);
}

static void detect_success_for_duplicate_faces(igl::opengl::ViewerData& viewer_data, Mesh& mesh,
	const defects::DuplicateFaces& duplicate_faces, const Eigen::RowVector3d& neutral_color, const Eigen::RowVector3d& defective_color)
{
	auto& face_colors = mesh.get_face_colors();
	face_colors.resizeLike(mesh.get_faces());
	const auto& face_indices = duplicate_faces.get_duplicate_face_indices();
	const auto& face_roots = duplicate_faces.get_duplicate_face_roots();
	Eigen::Index fi = 0, ri = 0;
	for (Eigen::Index i = 0; i < face_colors.rows(); ++i)
	{
		if (fi < face_indices.size() && face_indices[fi] == i)
		{
			face_colors.row(i) = defective_color;
			++fi;
		}
		else if (ri < face_roots.size() && face_roots[ri] == i)
		{
			face_colors.row(i) = defective_color;
			++ri;
		}
		else
			face_colors.row(i) = neutral_color;
	}
	viewer_data.set_colors(face_colors);
}

void GeoRepair::degenerate_faces_detect_success()
{
	const auto& degenerate_faces = defect_list.get<Defect::DEGENERATE_FACES>();
	detect_success_for_degenerate_vertex_patch(mesh_data(), mesh, degenerate_faces.get_degenerate_vertex_patch(), colors::vertex::neutral, colors::vertex::defective);
	detect_success_for_duplicate_faces(mesh_data(), mesh, degenerate_faces.get_duplicate_faces(), colors::face::neutral, colors::face::defective);
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::degenerate_vertex_patch_detect_success()
{
	detect_success_for_degenerate_vertex_patch(mesh_data(), mesh, defect_list.get<Defect::DEGENERATE_VERTEX_PATCH>(), colors::vertex::neutral, colors::vertex::defective);
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::duplicate_faces_detect_success()
{
	detect_success_for_duplicate_faces(mesh_data(), mesh, defect_list.get<Defect::DUPLICATE_FACES>(), colors::face::neutral, colors::face::defective);
	mesh.reset_vertex_colors();
	mesh_data().set_points(mesh.get_vertices(), mesh.get_vertex_colors());
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::general_duplicate_vertices_detect_success()
{
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resizeLike(mesh.get_vertices());
	const auto& defect = defect_list.get<Defect::GENERAL_DUPLICATE_VERTICES>();
	const auto& squared_distances = defect.get_squared_distances();
	double squared_tolerance = defect.tolerance * defect.tolerance;
	if (squared_tolerance > 0.0)
	{
		for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
		{
			auto it = squared_distances.find(i);
			if (it != squared_distances.end())
				vertex_colors.row(i) = colors::vertex::defective + it->second / squared_tolerance * (colors::vertex::defective_low - colors::vertex::defective);
			else
				vertex_colors.row(i) = colors::vertex::neutral;
		}
	}
	else
	{
		for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
		{
			if (squared_distances.count(i))
				vertex_colors.row(i) = colors::vertex::defective;
			else
				vertex_colors.row(i) = colors::vertex::neutral;
		}
	}

	mesh_data().set_points(mesh.get_vertices(), vertex_colors);
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::inverted_normals_detect_success()
{
	auto& face_colors = mesh.get_face_colors();
	face_colors = colors::face::neutral.replicate(mesh.get_faces().rows(), 1);
	const auto& inverted_normals = defect_list.get<Defect::INVERTED_NORMALS>();
	
	const auto& flip_faces = inverted_normals.get_flip_faces();
	for (const auto& flip_face_vector : flip_faces)
		for (Eigen::Index i : flip_face_vector)
			face_colors.row(i) = colors::face::defective;

	const auto& submesh_roots = inverted_normals.get_submesh_roots();
	for (Eigen::Index submesh_root : submesh_roots)
	{
		if (face_colors.row(submesh_root) == colors::face::neutral)
			face_colors.row(submesh_root) = colors::face::defective_alt;
		mesh.traverse_connected_submesh(submesh_root, [&](Eigen::Index, Eigen::Index f) {
			if (face_colors.row(f) == colors::face::neutral)
				face_colors.row(f) = colors::face::defective_alt;
			});
	}

	const auto& flip_entire_submesh_roots = inverted_normals.get_flip_entire_submesh_roots();
	for (Eigen::Index submesh_root : flip_entire_submesh_roots)
	{
		face_colors.row(submesh_root) = colors::face::defective;
		mesh.traverse_connected_submesh(submesh_root, [&](Eigen::Index, Eigen::Index f) {
			face_colors.row(f) = colors::face::defective;
			});
	}

	mesh_data().set_colors(face_colors);
	mesh.reset_vertex_colors();
	mesh_data().set_points(mesh.get_vertices(), mesh.get_vertex_colors());
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::noise_smoothing_detect_success()
{
	const auto& vertices = mesh.get_vertices();
	const auto& noisy_vertices = defect_list.get<Defect::NOISE_SMOOTHING>().get_noisy_vertices();
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resize(vertices.rows(), 3);
	Eigen::Index nv = 0;
	for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
	{
		if (nv < noisy_vertices.size() && noisy_vertices[nv] == i)
		{
			vertex_colors.row(i) = colors::vertex::defective;
			++nv;
		}
		else
			vertex_colors.row(i) = colors::vertex::neutral;
	}
	mesh_data().set_points(vertices, vertex_colors);
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::non_manifold_edges_detect_success()
{
	const auto& vertices = mesh.get_vertices();
	edge_data().clear_edges();

	const auto& defect = defect_list.get<Defect::NON_MANIFOLD_EDGES>();
	const auto& edges = defect.get_non_manifold_edges();
	const auto& counts = defect.get_edge_counts();
	double max_surplus = defect.get_max_edge_count() - 3;
	
	auto& E1 = mesh.get_edges_1();
	E1.resize(edges.size(), 3);
	auto& E2 = mesh.get_edges_2();
	E2.resize(edges.size(), 3);
	auto& C = mesh.get_edge_colors();
	C.resize(edges.size(), 3);
	
	Eigen::Index i = 0;
	for (auto iter = edges.begin(); iter != edges.end(); ++iter)
	{
		E1.row(i) = vertices.row(iter->v1);
		E2.row(i) = vertices.row(iter->v2);
		if (max_surplus > 1.0)
		{
			double factor = (counts.find(*iter)->second - 3) / max_surplus;
			C.row(i) = colors::edge::defective_alt + factor * (colors::edge::defective - colors::edge::defective_alt);
		}
		else
			C.row(i) = colors::edge::defective;
		++i;
	}

	edge_data().add_edges(E1, E2, C);
	mesh.reset_vertex_colors();
	mesh_data().set_points(vertices, mesh.get_vertex_colors());
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
}

void GeoRepair::null_faces_detect_success()
{
	const auto& vertices = mesh.get_vertices();
	const auto& faces = mesh.get_faces();
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors = colors::vertex::neutral.replicate(vertices.rows(), 1);
	edge_data().clear_edges();

	const auto& indices = defect_list.get<Defect::NULL_FACES>().get_null_face_indices();
	auto& E1 = mesh.get_edges_1();
	E1.resize(3 * indices.size(), 3);
	auto& E2 = mesh.get_edges_2();
	E2.resize(3 * indices.size(), 3);
	auto& EC = mesh.get_edge_colors();
	EC = colors::edge::defective.replicate(3 * indices.size(), 1);
	for (Eigen::Index i = 0; i < indices.size(); ++i)
	{
		Eigen::Index fi = indices[i];
		Eigen::RowVector3i face = faces.row(fi);
		E1.row(3 * i) = vertices.row(face(0));
		E1.row(3 * i + 1) = vertices.row(face(1));
		E1.row(3 * i + 2) = vertices.row(face(2));
		E2.row(3 * i) = vertices.row(face(1));
		E2.row(3 * i + 1) = vertices.row(face(2));
		E2.row(3 * i + 2) = vertices.row(face(0));
		if (face(0) == face(1) && face(0) == face(2))
		{
			vertex_colors.row(face(0)) = colors::vertex::defective;
			vertex_colors.row(face(1)) = colors::vertex::defective;
			vertex_colors.row(face(2)) = colors::vertex::defective;
		}
	}

	mesh_data().set_points(mesh.get_vertices(), vertex_colors);
	edge_data().add_edges(E1, E2, EC);
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
}

void GeoRepair::unbound_vertices_detect_success()
{
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resizeLike(mesh.get_vertices());
	const auto& unbound_vertices = defect_list.get<Defect::UNBOUND_VERTICES>().get_unbound_vertices();
	Eigen::Index uvx = 0;
	for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
	{
		if (uvx < unbound_vertices.size() && unbound_vertices[uvx] == i)
		{
			vertex_colors.row(i) = colors::vertex::defective;
			++uvx;
		}
		else
			vertex_colors.row(i) = colors::vertex::neutral;
	}
	mesh_data().set_points(mesh.get_vertices(), vertex_colors);
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
	mesh.reset_edge_colors();
	edge_data().clear_edges();
}

void GeoRepair::unpatched_holes_detect_success()
{
	edge_data().clear_edges();
	const auto& vertices = mesh.get_vertices();
	const auto& boundaries = defect_list.get<Defect::UNPATCHED_HOLES>().get_boundary_vertices();
	size_t num_boundary_vertices = 0;
	for (const auto& boundary : boundaries)
		num_boundary_vertices += boundary.size();

	auto& E1 = mesh.get_edges_1();
	E1.resize(num_boundary_vertices, 3);
	auto& E2 = mesh.get_edges_2();
	E2.resize(num_boundary_vertices, 3);
	auto& EC = mesh.get_edge_colors();
	EC = colors::edge::defective.replicate(num_boundary_vertices, 1);
	
	size_t offset = 0;
	for (const auto& boundary : boundaries)
	{
		for (size_t i = 0; i < boundary.size(); ++i)
		{
			E1.row(i + offset) = vertices.row(boundary[i]);
			if (i == 0)
				E2.row(i + offset) = vertices.row(boundary[boundary.size() - 1]);
			else
				E2.row(i + offset) = vertices.row(boundary[i - 1]);
		}
		offset += boundary.size();
	}
	
	edge_data().add_edges(E1, E2, EC);
	mesh.reset_vertex_colors();
	mesh_data().set_points(vertices, mesh.get_vertex_colors());
	mesh.reset_face_colors();
	mesh_data().set_colors(mesh.get_face_colors());
}

void GeoRepair::reset_colors()
{
	mesh_data().set_points(mesh.get_vertices(), colors::vertex::neutral);
	mesh_data().set_colors(colors::face::neutral.replicate(mesh.get_faces().rows(), 1));
	edge_data().clear_edges();
}
