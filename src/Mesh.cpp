#include "Mesh.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>

#include <stack>

#include "defects/Common.h"
#include "GeoRepair.h"

bool Mesh::load(const char* filename)
{
	data->primary.mesh_filename = filename;

	// TODO eventually, add support for texture coordinates, etc., for specific file extensions
	if (!igl::read_triangle_mesh(filename, data->primary.V, data->primary.F))
		return false;
	reset_vertex_colors();
	reset_face_colors();
	reset_edge_colors();

	// remove invalid faces
	Eigen::Index max_vertex = data->primary.V.rows() - 1;
	remove_rows(data->primary.F, [max_vertex](Eigen::Index i, const Eigen::RowVector3i& face) {
		return std::max({ face(0), face(1), face(2) }) > max_vertex || std::min({ face(0), face(1), face(2) }) < 0;
		});
	push();
	return true;
}

bool Mesh::save(const char* filename)
{
	return igl::write_triangle_mesh(filename, data->primary.V, data->primary.F);
}

void Mesh::push()
{
	auto old_data = data;
	history.push(old_data);
	data = std::make_shared<MeshData>();
	*data = *old_data;
}

bool Mesh::undo()
{
	auto new_data = history.undo();
	if (new_data && new_data != data)
	{
		*data = *new_data;
		return true;
	}
	return false;
}

bool Mesh::redo()
{
	auto new_data = history.redo();
	if (new_data && new_data != data)
	{
		*data = *new_data;
		return true;
	}
	return false;
}

void Mesh::reset_vertex_colors()
{
	data->primary.VC.resize(1, 3);
	data->primary.VC.row(0) = colors::vertex::neutral;
}

void Mesh::reset_face_colors()
{
	data->primary.FC.resize(1, 3);
	data->primary.FC.row(0) = colors::face::neutral;
}

void Mesh::reset_edge_colors()
{
	data->primary.E1.resize(0, 0);
	data->primary.E2.resize(0, 0);
	data->primary.EC.resize(0, 0);
}

static void traverse_submesh_dfs(const Eigen::SparseMatrix<int>& fadj, Eigen::Index start_face, std::vector<bool>& visited_faces, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process)
{
	std::stack<Eigen::Index> stack;
	stack.push(start_face);
	visited_faces[start_face] = true;

	while (!stack.empty())
	{
		Eigen::Index face_index = stack.top();
		stack.pop();

		for (Eigen::SparseMatrix<int>::InnerIterator iter(fadj, face_index); iter; ++iter)
		{
			Eigen::Index neighbour_face = iter.index();
			if (!visited_faces[neighbour_face])
			{
				visited_faces[neighbour_face] = true;
				adj_process(face_index, neighbour_face);
				stack.push(neighbour_face);
			}
		}
	}
}

void Mesh::traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const
{
	const auto& FADJ = data->get_fadj();
	std::vector<bool> visited_faces(data->primary.F.rows(), false);
	for (Eigen::Index i = 0; i < visited_faces.size(); ++i)
	{
		if (!visited_faces[i])
		{
			root_process(i);
			traverse_submesh_dfs(FADJ, i, visited_faces, adj_process);
		}
	}
}

void Mesh::traverse_connected_submesh(Eigen::Index submesh_root, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const
{
	std::vector<bool> visited_faces(data->primary.F.rows(), false);
	traverse_submesh_dfs(data->get_fadj(), submesh_root, visited_faces, adj_process);
}
