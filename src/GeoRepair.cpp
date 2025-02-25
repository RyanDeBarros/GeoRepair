#define IGL_VIEWER_VIEWER_QUIET
// TODO add above macro to laplacian-deformation

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

	void reset_colors();

	struct
	{
		ImVec4 detected = ImVec4(0.3f, 0.65f, 0.35f, 1.0f);
	} style_colors;

	struct
	{
		Eigen::RowVector3d neutral = Eigen::RowVector3d(0.5, 0.5, 0.5);
		Eigen::RowVector3d defective = Eigen::RowVector3d(0.9, 0.3, 0.3);
		Eigen::RowVector3d defective_low = Eigen::RowVector3d(0.7, 0.3, 0.6);
	} mesh_vertex_colors;

	struct
	{
		Eigen::RowVector3d neutral = Eigen::RowVector3d(1.0, 0.894, 0.235);
		Eigen::RowVector3d defective = Eigen::RowVector3d(1.0, 0.0, 0.784);
	} mesh_face_colors;
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
	viewer.data().point_size = 8.0f;
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
	data.set_points(mesh.get_vertices(), mesh.get_vertex_colors());
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
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("Load mesh", "Ctrl+O"))
				load_mesh();
			if (ImGui::MenuItem("Save mesh", "Ctrl+S"))
				save_mesh();
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Edit"))
		{
			if (ImGui::MenuItem("Undo", "Ctrl+Z"))
				undo();
			if (ImGui::MenuItem("Redo", "Ctrl+Shft+Z"))
				redo();
			ImGui::Separator();
			if (ImGui::MenuItem("Clear visualization"))
				reset_colors();
			if (ImGui::MenuItem("Restore visualization"))
			{
				viewer.data().set_points(mesh.get_vertices(), mesh.get_vertex_colors());
				// TODO set face/edge colors -> store face/edge colors in primary data
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}
}

#define render_defect_gui_header(defect)\
	bool detected = defect_list[defect].in_detected_state();\
	if (detected) ImGui::PushStyleColor(ImGuiCol_Header, style_colors.detected);\
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

		// TODO noise smoothing parameters

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
			non_manifold_edges.detect(mesh);
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
			non_manifold_edges.reset();
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
	{
		defect_list[defect].reset();
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

void GeoRepair::degenerate_faces_detect_success()
{
	// TODO
}

void GeoRepair::degenerate_vertex_patch_detect_success()
{
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resizeLike(mesh.get_vertices());
	auto& vertex_clusters = defect_list.get<Defect::DEGENERATE_VERTEX_PATCH>().get_vertex_clusters();
	for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
	{
		if (vertex_clusters.exists(i))
			vertex_colors.row(i) = mesh_vertex_colors.defective;
		else
			vertex_colors.row(i) = mesh_vertex_colors.neutral;
	}
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
}

void GeoRepair::duplicate_faces_detect_success()
{
	Eigen::MatrixXd face_colors(mesh.get_faces().rows(), 3);
	const auto& duplicate_faces = defect_list.get<Defect::DUPLICATE_FACES>();
	const auto& face_indices = duplicate_faces.get_duplicate_face_indices();
	const auto& face_roots = duplicate_faces.get_duplicate_face_roots();
	Eigen::Index fi = 0, ri = 0;
	for (Eigen::Index i = 0; i < face_colors.rows(); ++i)
	{
		if (fi < face_indices.size() && face_indices[fi] == i)
		{
			face_colors.row(i) = mesh_face_colors.defective;
			++fi;
		}
		else if (ri < face_roots.size() && face_roots[ri] == i)
		{
			face_colors.row(i) = mesh_face_colors.defective;
			++ri;
		}
		else
			face_colors.row(i) = mesh_face_colors.neutral;
	}
	viewer.data().set_colors(face_colors);
}

void GeoRepair::general_duplicate_vertices_detect_success()
{
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resizeLike(mesh.get_vertices());
	const auto& defect = defect_list.get<Defect::GENERAL_DUPLICATE_VERTICES>();
	const auto& squared_distances = defect.get_squared_distances();
	double squared_tolerance = defect.tolerance * defect.tolerance;
	
	for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
	{
		auto it = squared_distances.find(i);
		if (it != squared_distances.end())
			vertex_colors.row(i) = mesh_vertex_colors.defective + it->second / squared_tolerance * (mesh_vertex_colors.defective_low - mesh_vertex_colors.defective);
		else
			vertex_colors.row(i) = mesh_vertex_colors.neutral;
	}
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
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
	auto& vertex_colors = mesh.get_vertex_colors();
	vertex_colors.resizeLike(mesh.get_vertices());
	const auto& unbound_vertices = defect_list.get<Defect::UNBOUND_VERTICES>().get_unbound_vertices();
	Eigen::Index uvx = 0;
	for (Eigen::Index i = 0; i < vertex_colors.rows(); ++i)
	{
		if (uvx < unbound_vertices.size() && unbound_vertices[uvx] == i)
		{
			vertex_colors.row(i) = mesh_vertex_colors.defective;
			++uvx;
		}
		else
			vertex_colors.row(i) = mesh_vertex_colors.neutral;
	}
	viewer.data().set_points(mesh.get_vertices(), vertex_colors);
}

void GeoRepair::unpatched_holes_detect_success()
{
	// TODO
}

void GeoRepair::reset_colors()
{
	viewer.data().set_points(mesh.get_vertices(), mesh_vertex_colors.neutral);
	viewer.data().clear_edges();
	viewer.data().set_colors(mesh_face_colors.neutral.replicate(mesh.get_faces().rows(), 1));
}
