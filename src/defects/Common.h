#pragma once

#include "../Mesh.h"
#include "../EquivalenceClasses.h"

namespace defects
{
	struct DefectBase
	{
		void detect(const Mesh& mesh);
		void repair(Mesh& mesh);
		virtual void reset() = 0;
		virtual bool in_detected_state() const = 0;
		virtual void _detect(const Mesh&) = 0;
		virtual void _repair(Mesh&) = 0;
	};
}

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

extern double area(Eigen::RowVector3d v0, Eigen::RowVector3d v1, Eigen::RowVector3d v2);
extern double area(const Mesh& mesh, Eigen::Index face);
extern double signed_volume(const Mesh& mesh, Eigen::Index submesh_root);
extern Eigen::Vector3d face_normal(const Mesh& mesh, Eigen::Index face_index);
extern Eigen::RowVector3i rotate_forward(Eigen::RowVector3i vector);
extern Eigen::RowVector3i rotate_backward(Eigen::RowVector3i vector);
extern void rotate_forward_in_place(Eigen::RowVector3i& vector);
extern void rotate_backward_in_place(Eigen::RowVector3i& vector);
extern bool same_adjacent_winding_order(Eigen::RowVector3i face1, Eigen::RowVector3i face2);
extern void standard_deviation(const Eigen::VectorXd& vec, double& mean, double& stddev);
extern bool on_edge(Eigen::RowVector3d query, Eigen::RowVector3d v1, Eigen::RowVector3d v2);
extern bool on_triangle(Eigen::RowVector3d query, Eigen::RowVector3d v1, Eigen::RowVector3d v2, Eigen::RowVector3d v3);
extern bool on_triangle_boundary(Eigen::RowVector3d query, Eigen::RowVector3d v1, Eigen::RowVector3d v2, Eigen::RowVector3d v3);

extern void remove_rows(Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted);
extern void remove_rows(Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, bool indices_sorted);
extern void remove_rows_setup(const Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index, const Eigen::RowVector3i&)>& remove_row_predicate);
extern void remove_rows(Eigen::MatrixXi& mat, const std::function<bool(Eigen::Index, const Eigen::RowVector3i&)>& remove_row_predicate);
extern void remove_rows(Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted);
extern void remove_rows(Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, bool indices_sorted);
extern void remove_rows_setup(const Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index, const Eigen::RowVector3d&)>& remove_row_predicate);
extern void remove_rows(Eigen::MatrixXd& mat, const std::function<bool(Eigen::Index, const Eigen::RowVector3d&)>& remove_row_predicate);

extern void reindex_faces(Eigen::MatrixXi& faces, const std::vector<Eigen::Index>& removed_vertices);

template<typename T>
inline void resize_vector_with_value(std::vector<T>& vector, size_t size, T value)
{
	size_t old_size = vector.size();
	vector.resize(size, value);
	if (size > old_size)
		std::fill(vector.begin(), vector.begin() + old_size, value);
	else
		std::fill(vector.begin(), vector.end(), value);
}
