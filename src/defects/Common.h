#pragma once

#include "../MeshData.h"

struct EigenMatrixHash
{
	template<typename Derived>
	size_t operator()(const Eigen::MatrixBase<Derived>& matrix) const
	{
		size_t hash = 0;
		for (int i = 0; i < matrix.size(); ++i)
			hash ^= std::hash<typename Derived::Scalar>{}(matrix(i)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		return hash;
	}
};

extern double area(const Eigen::RowVector3d& v0, const Eigen::RowVector3d& v1, const Eigen::RowVector3d& v2);

extern void remove_rows(Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted);
extern void remove_rows_setup(const Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index)>& remove_row_predicate);
extern void remove_rows(Eigen::MatrixXi& mat, const std::function<bool(Eigen::Index)>& remove_row_predicate);
extern void remove_rows(Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted);
extern void remove_rows_setup(const Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index)>& remove_row_predicate);
extern void remove_rows(Eigen::MatrixXd& mat, const std::function<bool(Eigen::Index)>& remove_row_predicate);

extern void reindex_faces(Eigen::MatrixXi& faces, const std::vector<Eigen::Index>& removed_vertices);
