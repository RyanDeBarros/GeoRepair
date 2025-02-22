#include "DuplicateFaces.h"

#include <unordered_set>

void defects::DuplicateFaces::_detect(const MeshData& mesh)
{
	reset();
	const Eigen::MatrixXi& faces = mesh.get_faces();
	std::unordered_set<Eigen::RowVector3i, EigenMatrixHash> existing_faces;
	if (ignore_normals)
	{
		remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&](Eigen::Index i) {
			bool unique = existing_faces.insert(Eigen::RowVector3i(faces(i, 0), faces(i, 1), faces(i, 2))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 2), faces(i, 0), faces(i, 1))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 1), faces(i, 2), faces(i, 0))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 2), faces(i, 1), faces(i, 0))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 0), faces(i, 2), faces(i, 1))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 1), faces(i, 0), faces(i, 2))).second;
			return !unique;
			});
	}
	else
	{
		remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&](Eigen::Index i) {
			bool unique = existing_faces.insert(Eigen::RowVector3i(faces(i, 0), faces(i, 1), faces(i, 2))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 2), faces(i, 0), faces(i, 1))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(faces(i, 1), faces(i, 2), faces(i, 0))).second;
			return !unique;
			});
	}
}

void defects::DuplicateFaces::_repair(MeshData& mesh)
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
