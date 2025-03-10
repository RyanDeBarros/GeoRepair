#include "InvertedNormals.h"

void defects::InvertedNormals::_detect(const Mesh& mesh)
{
	const auto& faces = mesh.get_faces();
	resize_vector_with_value(winding_adjustments, faces.rows(), WindingAdjustment::UNPROCESSED);
	const auto& all_submeshes = mesh.get_connected_submeshes();
	for (const auto& submesh : all_submeshes)
	{
		winding_adjustments[submesh[0]] = WindingAdjustment::KEEP;
		std::vector<Eigen::Index> flip_faces_in_submesh;
		mesh.traverse_connected_submesh(submesh[0], [&](Eigen::Index face1, Eigen::Index face2) {
			if (same_adjacent_winding_order(faces.row(face1), faces.row(face2)))
				winding_adjustments[face2] = winding_adjustments[face1] == WindingAdjustment::KEEP ? WindingAdjustment::KEEP : WindingAdjustment::FLIP;
			else
				winding_adjustments[face2] = winding_adjustments[face1] == WindingAdjustment::KEEP ? WindingAdjustment::FLIP : WindingAdjustment::KEEP;
			if (winding_adjustments[face2] == WindingAdjustment::FLIP)
				flip_faces_in_submesh.push_back(face2);
			});
		// assume submesh is closed and flip normals if they are pointing inwards. if the submesh is open, then normals being inwards/outwards is undefined.
		if (flip_faces_in_submesh.empty())
		{
			if (signed_volume(mesh, submesh[0]) < 0.0)
				flip_entire_submesh_roots.push_back(submesh[0]);
		}
		else
		{
			flip_faces.push_back(std::move(flip_faces_in_submesh));
			submesh_roots.push_back(submesh[0]);
		}
	}
}

void defects::InvertedNormals::_repair(Mesh& mesh)
{
	auto& faces = mesh.get_faces();
	for (size_t i = 0; i < flip_faces.size(); ++i)
	{
		const auto& flip_faces_in_submesh = flip_faces[i];
		for (Eigen::Index face_index : flip_faces_in_submesh)
			faces.row(face_index).reverseInPlace();
		if (signed_volume(mesh, submesh_roots[i]) < 0.0)
			flip(mesh, submesh_roots[i]);
	}

	for (Eigen::Index submesh_root : flip_entire_submesh_roots)
		flip(mesh, submesh_root);
}

void defects::InvertedNormals::reset()
{
	flip_faces.clear();
	submesh_roots.clear();
	flip_entire_submesh_roots.clear();
}

bool defects::InvertedNormals::in_detected_state() const
{
	return !flip_faces.empty() || !flip_entire_submesh_roots.empty();
}

void defects::InvertedNormals::flip(Mesh& mesh)
{
	mesh.get_faces().rowwise().reverseInPlace();
}

void defects::InvertedNormals::flip(Mesh& mesh, Eigen::Index submesh_root)
{
	auto& faces = mesh.get_faces();
	const auto& submesh = mesh.get_connected_submesh(submesh_root);
	for (Eigen::Index face : submesh)
		faces.row(face).reverseInPlace();
}
