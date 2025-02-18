#include "DuplicateFaces.h"

#include <unordered_set>

void defects::DuplicateFaces::detect(const MeshData& mesh)
{
	const Eigen::MatrixXi& faces = mesh.get_faces();
	std::unordered_set<Eigen::RowVectorXi, EigenMatrixHash> existing_faces;
	for (Eigen::Index i = 0; i < faces.rows(); ++i)
	{
		if (!existing_faces.insert(faces.row(i)).second)
		{
			if (duplicate_face_indices.empty())
				maximum_block_height = i;
			else
				maximum_block_height = std::max(maximum_block_height, i - duplicate_face_indices.back() - 1);
			duplicate_face_indices.push_back(i);
		}
	}
}

void defects::DuplicateFaces::repair(MeshData& mesh)
{
	if (duplicate_face_indices.empty())
		return;
	Eigen::MatrixXi& faces = mesh.get_faces();
	Eigen::MatrixXi temp(maximum_block_height, faces.cols());
	Eigen::Index new_starting_row = 0;
	for (int i = 0; i < duplicate_face_indices.size() + 1; ++i)
	{
		Eigen::Index num_rows;
		if (i == 0)
			num_rows = duplicate_face_indices[0];
		else if (i == duplicate_face_indices.size())
			num_rows = faces.rows() - duplicate_face_indices[i - 1] - 1;
		else
			num_rows = duplicate_face_indices[i] - duplicate_face_indices[i - 1] - 1;
		temp.block(0, 0, num_rows, faces.cols()) = faces.block(new_starting_row + i, 0, num_rows, faces.cols());
		faces.block(new_starting_row, 0, num_rows, faces.cols()) = temp.block(0, 0, num_rows, faces.cols());
		new_starting_row += num_rows;
	}
	faces.conservativeResize(faces.rows() - duplicate_face_indices.size(), faces.cols());
}

void defects::DuplicateFaces::reset()
{
	duplicate_face_indices.clear();
	maximum_block_height = 0;
}
