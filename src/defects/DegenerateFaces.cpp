#include "DegenerateFaces.h"

void defects::DegenerateFaces::_detect(const Mesh& mesh)
{
	degenerate_vertex_patch.tolerance = tolerance;
	degenerate_vertex_patch.detect(mesh);
	duplicate_faces.ignore_normals = ignore_normals;
	duplicate_faces.detect(mesh);
}

void defects::DegenerateFaces::_repair(Mesh& mesh)
{
	if (degenerate_vertex_patch.in_detected_state())
	{
		degenerate_vertex_patch._repair(mesh);
		duplicate_faces.detect(mesh); // re-detect, since degenerate_vertex_patch may have introduced new duplicate faces. it's assumed that ignore_normals did not change
	}
	if (duplicate_faces.in_detected_state())
		duplicate_faces._repair(mesh);
}

void defects::DegenerateFaces::reset()
{
	degenerate_vertex_patch.reset();
	duplicate_faces.reset();
}

bool defects::DegenerateFaces::in_detected_state() const
{
	return degenerate_vertex_patch.in_detected_state() || duplicate_faces.in_detected_state();
}
