#include "NullFaces.h"

void defects::NullFaces::_detect(const Mesh& mesh)
{
	const Eigen::MatrixXi& faces = mesh.get_faces();
	remove_rows_setup(faces, null_face_indices, maximum_block_height, [&](Eigen::Index i, const Eigen::Vector3i& face) {
		Eigen::Index v0 = face(0);
		Eigen::Index v1 = face(1);
		Eigen::Index v2 = face(2);
		return v0 == v1 || v0 == v2 || v1 == v2;
		});
}

void defects::NullFaces::_repair(Mesh& mesh)
{
	remove_rows(mesh.get_faces(), null_face_indices, maximum_block_height, true);
	mesh.reset_face_colors();
}

void defects::NullFaces::reset()
{
	null_face_indices.clear();
	maximum_block_height = 0;
}

bool defects::NullFaces::in_detected_state() const
{
	return !null_face_indices.empty();
}
