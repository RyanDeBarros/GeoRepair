#include "Mesh.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>

#include "defects/Common.h"

// TODO not only are texture coordinates, materials, etc. lost after loading the mesh, so are faceless edges.
bool Mesh::load(const char* filename)
{
	data->mesh_filename = filename;

	// TODO eventually, add support for texture coordinates, etc., for specific file extensions
	if (!igl::read_triangle_mesh(filename, data->V, data->F))
		return false;
	data->VC.resize(1, 3);
	data->VC.row(0) = Eigen::RowVector3d(0.5, 0.5, 0.5);

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
	auto old_data = data;
	history.push(old_data);
	data = std::make_shared<MeshPrimaryData>();
	*data = *old_data;
	aux.reset(); // TODO no need to reset if storing aux data in history
}

bool Mesh::undo()
{
	auto new_data = history.undo();
	if (new_data && new_data != data)
	{
		*data = *new_data;
		aux.reset(); // TODO no need to reset if storing aux data in history
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
		aux.reset(); // TODO no need to reset if storing aux data in history
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
