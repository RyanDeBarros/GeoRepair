#include "Mesh.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>

#include "defects/Common.h"

bool Mesh::load(const char* filename)
{
	// TODO eventually, add support for texture coordinates, etc., for specific file extensions
	if (!igl::read_triangle_mesh(filename, data->V, data->F))
		return false;
	// remove invalid faces
	Eigen::Index max_vertex = data->V.rows() - 1;
	remove_rows(data->F, [max_vertex](Eigen::Index i, const Eigen::RowVector3i& face) {
		return std::max({ face(0), face(1), face(2) }) > max_vertex || std::min({ face(0), face(1), face(2) }) < 0;
		});
	push();
	return true;
}

bool Mesh::save(const char* filename)
{
	return igl::write_triangle_mesh(filename, data->V, data->F);
}

void Mesh::push()
{
	auto old_data = std::make_shared<MeshPrimaryData>();
	old_data->V = data->V;
	old_data->F = data->F;
	history.push(old_data);
	aux.reset();
}

bool Mesh::undo()
{
	if (auto new_data = history.undo())
	{
		data = std::move(new_data);
		return true;
	}
	return false;
}

bool Mesh::redo()
{
	if (auto new_data = history.redo())
	{
		data = std::move(new_data);
		return true;
	}
	return false;
}

static void traverse_submesh_dfs(const Eigen::SparseMatrix<int>& fadj, Eigen::Index face_index, std::vector<bool>& visited_faces, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process)
{
	visited_faces[face_index] = true;
	for (Eigen::SparseMatrix<int>::InnerIterator iter(fadj, face_index); iter; ++iter)
	{
		Eigen::Index neighbour_face = iter.index();
		if (!visited_faces[neighbour_face])
		{
			adj_process(face_index, neighbour_face);
			traverse_submesh_dfs(fadj, neighbour_face, visited_faces, adj_process);
		}
	}
}

void Mesh::traverse_connected_submeshes(const std::function<void(Eigen::Index face)>& root_process, const std::function<void(Eigen::Index face1, Eigen::Index face2)>& adj_process) const
{
	const auto& FADJ = aux.get_fadj(*data);
	std::vector<bool> visited_faces(data->F.rows(), false);
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
	std::vector<bool> visited_faces(data->F.rows(), false);
	traverse_submesh_dfs(aux.get_fadj(*data), submesh_root, visited_faces, adj_process);
}
