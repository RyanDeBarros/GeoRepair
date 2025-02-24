#include "DuplicateFaces.h"

#include <unordered_set>

void defects::DuplicateFaces::_detect(const Mesh& mesh)
{
	const Eigen::MatrixXi& faces = mesh.get_faces();
	std::unordered_set<Eigen::RowVector3i, EigenMatrixHash> existing_faces;
	if (ignore_normals)
	{
		remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&existing_faces](Eigen::Index i, const Eigen::RowVector3i& face) {
			bool unique = existing_faces.insert(Eigen::RowVector3i(face(0), face(1), face(2))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(2), face(0), face(1))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(1), face(2), face(0))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(2), face(1), face(0))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(0), face(2), face(1))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(1), face(0), face(2))).second;
			return !unique;
			});
	}
	else
	{
		remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&existing_faces](Eigen::Index i, const Eigen::RowVector3i& face) {
			bool unique = existing_faces.insert(Eigen::RowVector3i(face(0), face(1), face(2))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(2), face(0), face(1))).second;
			unique &= existing_faces.insert(Eigen::RowVector3i(face(1), face(2), face(0))).second;
			return !unique;
			});
	}
}

void defects::DuplicateFaces::_repair(Mesh& mesh)
{
	remove_rows(mesh.get_faces(), duplicate_face_indices, maximum_block_height, true);
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
