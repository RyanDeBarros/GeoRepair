#include "Common.h"

#include <unordered_map>

double area(const Eigen::RowVector3d& v0, const Eigen::RowVector3d& v1, const Eigen::RowVector3d& v2)
{
    return 0.5 * ((v1 - v0).cross(v2 - v0)).norm();
}

void remove_rows(Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted)
{
	if (!indices_sorted)
		std::sort(indices.begin(), indices.end());
	Eigen::MatrixXi temp(maximum_block_height, mat.cols());
	Eigen::Index new_starting_row = 0;
	for (Eigen::Index i = 0; i < indices.size() + 1; ++i)
	{
		Eigen::Index num_rows;
		if (i == 0)
			num_rows = indices[0];
		else if (i == indices.size())
			num_rows = mat.rows() - indices[i - 1] - 1;
		else
			num_rows = indices[i] - indices[i - 1] - 1;
		temp.block(0, 0, num_rows, mat.cols()) = mat.block(new_starting_row + i, 0, num_rows, mat.cols());
		mat.block(new_starting_row, 0, num_rows, mat.cols()) = temp.block(0, 0, num_rows, mat.cols());
		new_starting_row += num_rows;
	}
	mat.conservativeResize(mat.rows() - indices.size(), mat.cols());
}

void remove_rows(Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, bool indices_sorted)
{
	remove_rows(mat, indices, mat.rows() - indices.size(), indices_sorted);
}

void remove_rows_setup(const Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index)>& remove_row_predicate)
{
	indices.clear();
	maximum_block_height = 0;
	for (Eigen::Index i = 0; i < mat.rows(); ++i)
	{
		if (remove_row_predicate(i))
		{
			if (indices.empty())
				maximum_block_height = i;
			else
				maximum_block_height = std::max(maximum_block_height, i - indices.back() - 1);
			indices.push_back(i);
		}
	}
	maximum_block_height = std::max(maximum_block_height, mat.rows() - indices.back() - 1);
}

void remove_rows(Eigen::MatrixXi& mat, const std::function<bool(Eigen::Index)>& remove_row_predicate)
{
	std::vector<Eigen::Index> indices;
	Eigen::Index maximum_block_height;
	remove_rows_setup(mat, indices, maximum_block_height, remove_row_predicate);
	remove_rows(mat, indices, maximum_block_height, true);
}

void remove_rows(Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted)
{
	if (!indices_sorted)
		std::sort(indices.begin(), indices.end());
	Eigen::MatrixXd temp(maximum_block_height, mat.cols());
	Eigen::Index new_starting_row = 0;
	for (Eigen::Index i = 0; i < indices.size() + 1; ++i)
	{
		Eigen::Index num_rows;
		if (i == 0)
			num_rows = indices[0];
		else if (i == indices.size())
			num_rows = mat.rows() - indices[i - 1] - 1;
		else
			num_rows = indices[i] - indices[i - 1] - 1;
		temp.block(0, 0, num_rows, mat.cols()) = mat.block(new_starting_row + i, 0, num_rows, mat.cols());
		mat.block(new_starting_row, 0, num_rows, mat.cols()) = temp.block(0, 0, num_rows, mat.cols());
		new_starting_row += num_rows;
	}
	mat.conservativeResize(mat.rows() - indices.size(), mat.cols());
}

void remove_rows(Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, bool indices_sorted)
{
	remove_rows(mat, indices, mat.rows() - indices.size(), indices_sorted);
}

void remove_rows_setup(const Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index)>& remove_row_predicate)
{
	indices.clear();
	maximum_block_height = 0;
	for (Eigen::Index i = 0; i < mat.rows(); ++i)
	{
		if (remove_row_predicate(i))
		{
			if (indices.empty())
				maximum_block_height = i;
			else
				maximum_block_height = std::max(maximum_block_height, i - indices.back() - 1);
			indices.push_back(i);
		}
	}
	maximum_block_height = std::max(maximum_block_height, mat.rows() - indices.back() - 1);
}

void remove_rows(Eigen::MatrixXd& mat, const std::function<bool(Eigen::Index)>& remove_row_predicate)
{
	std::vector<Eigen::Index> indices;
	Eigen::Index maximum_block_height;
	remove_rows_setup(mat, indices, maximum_block_height, remove_row_predicate);
	remove_rows(mat, indices, maximum_block_height, true);
}

void reindex_faces(Eigen::MatrixXi& faces, const std::vector<Eigen::Index>& removed_vertices)
{
	std::unordered_map<Eigen::Index, Eigen::Index> face_map;
	for (Eigen::Index i = 0; i < faces.size(); ++i)
	{
		auto prior_vertex = faces(i);
		auto it = face_map.find(prior_vertex);
		if (it != face_map.end())
			faces(i) = it->second;
		else
		{
			auto lb = std::lower_bound(removed_vertices.begin(), removed_vertices.end(), prior_vertex);
			auto offset = lb - removed_vertices.begin();
			if (lb != removed_vertices.end() && *lb == prior_vertex)
				++offset;
			auto post_vertex = prior_vertex - offset;
			faces(i) = post_vertex;
			face_map[prior_vertex] = post_vertex;
		}
	}
}
