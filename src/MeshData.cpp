#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/adjacency_list.h>
#include <igl/vertex_triangle_adjacency.h>

// TODO disclaimer that triangulation will occur, and no information (for now, anyways) about texture coordinates, materials, etc., will be retained.

bool MeshData::load(const char* filename)
{
	// TODO eventually, add support for texture coordinates, etc., for specific file extensions
	if (!igl::read_triangle_mesh(filename, V, F))
		return false;
	refresh();
	return true;
}

bool MeshData::save(const char* filename)
{
	return igl::write_triangle_mesh(filename, V, F);
}

void MeshData::refresh()
{
	tree.init(V, F);
	igl::adjacency_list(F, VADJ);
	igl::vertex_triangle_adjacency(V.rows(), F, FADJ_row, FADJ_col);
}
