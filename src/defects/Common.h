#pragma once

#include "../MeshData.h"

struct EigenMatrixHash
{
	template<typename Derived>
	size_t operator()(const Eigen::MatrixBase<Derived>& matrix) const
	{
		size_t hash;
		for (int i = 0; i < matrix.size(); ++i)
			hash ^= std::hash<typename Derived::Scalar>{}(matrix(i)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		return hash;
	}
};
