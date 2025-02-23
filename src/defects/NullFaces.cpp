#include "NullFaces.h"

void defects::NullFaces::_detect(const Mesh& mesh)
{
	const Eigen::MatrixXi& faces = mesh.get_faces();
	remove_rows_setup(faces, null_face_indices, maximum_block_height, [&](Eigen::Index i) {
		auto v0 = faces(i, 0);
		auto v1 = faces(i, 1);
		auto v2 = faces(i, 2);
		return v0 == v1 || v0 == v2 || v1 == v2;
		});
}

void defects::NullFaces::_repair(Mesh& mesh)
{
	remove_rows(mesh.get_faces(), null_face_indices, maximum_block_height, true);
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
