#include "DuplicateFaces.h"

#include <unordered_set>

void defects::DuplicateFaces::detect(const MeshData& mesh)
{
	reset();
	const Eigen::MatrixXi& faces = mesh.get_faces();
	std::unordered_set<Eigen::RowVectorXi, EigenMatrixHash> existing_faces;
	remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&](Eigen::Index i) {
		return !existing_faces.insert(faces.row(i)).second;
		});
}

void defects::DuplicateFaces::repair(MeshData& mesh)
{
	if (in_detected_state())
	{
		remove_rows(mesh.get_faces(), duplicate_face_indices, maximum_block_height, true);
		reset();
	}
}

void defects::DuplicateFaces::reset()
{
	duplicate_face_indices.clear();
	maximum_block_height = 0;
}

bool defects::DuplicateFaces::in_detected_state() const
{
	return !duplicate_face_indices.empty();
}
