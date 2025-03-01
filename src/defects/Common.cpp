#include "Common.h"

#include <unordered_map>

void defects::DefectBase::detect(const Mesh& mesh)
{
	reset();
	_detect(mesh);
}

void defects::DefectBase::repair(Mesh& mesh)
{
	if (in_detected_state())
	{
		_repair(mesh);
		reset();
		mesh.push();
	}
}

double area(Eigen::RowVector3d v0, Eigen::RowVector3d v1, Eigen::RowVector3d v2)
{
    return 0.5 * ((v1 - v0).cross(v2 - v0)).norm();
}

double area(const Mesh& mesh, Eigen::Index face)
{
	const auto& vertices = mesh.get_vertices();
	const auto& f = mesh.get_faces().row(face);
	return area(vertices.row(f(0)), vertices.row(f(1)), vertices.row(f(2)));
}

double signed_volume(const Mesh& mesh, Eigen::Index submesh_root)
{
	const auto& vertices = mesh.get_vertices();
	const auto& faces = mesh.get_faces();
	const auto& submesh = mesh.get_connected_submesh(submesh_root);
	double signed_volume = 0.0;
	for (Eigen::Index face : submesh)
	{
		Eigen::Vector3d v0 = vertices.row(faces(face, 0));
		Eigen::Vector3d v1 = vertices.row(faces(face, 1));
		Eigen::Vector3d v2 = vertices.row(faces(face, 2));
		signed_volume += v0.cross(v1).dot(v2);
	}
	return signed_volume / 6.0;
}

Eigen::Vector3d face_normal(const Mesh& mesh, Eigen::Index face_index)
{
	const auto& vertices = mesh.get_vertices();
	const auto& faces = mesh.get_faces();
	Eigen::Vector3d p0 = vertices.row(faces(face_index, 0));
	Eigen::Vector3d p1 = vertices.row(faces(face_index, 1));
	Eigen::Vector3d p2 = vertices.row(faces(face_index, 2));
	return (p1 - p0).cross(p2 - p0);
}

Eigen::RowVector3i rotate_forward(Eigen::RowVector3i vector)
{
	return Eigen::RowVector3i(vector(2), vector(0), vector(1));
}

Eigen::RowVector3i rotate_backward(Eigen::RowVector3i vector)
{
	return Eigen::RowVector3i(vector(1), vector(2), vector(0));
}

void rotate_forward_in_place(Eigen::RowVector3i& vector)
{
	int temp;
	std::swap(vector(0), temp);
	std::swap(vector(1), temp);
	std::swap(vector(2), temp);
	std::swap(vector(0), temp);
}

void rotate_backward_in_place(Eigen::RowVector3i& vector)
{
	int temp;
	std::swap(vector(2), temp);
	std::swap(vector(1), temp);
	std::swap(vector(0), temp);
	std::swap(vector(2), temp);
}

bool same_adjacent_winding_order(Eigen::RowVector3i face1, Eigen::RowVector3i face2)
{
	// a b * | a b * --> false
	// a b * | b * a --> false
	// a b * | * a b --> false
	// a b * | a * b --> true
	// a b * | b a * --> true
	// a b * | * b a --> true
	// a * b | a * b --> false
	// a * b | b a * --> false
	// a * b | * b a --> false
	// a * b | a b * --> true
	// a * b | b * a --> true
	// a * b | * a b --> true
	if (face1(0) != face2(0))
	{
		if (face1(0) == face2(1))
			rotate_backward_in_place(face2);
		else if (face1(0) == face2(2))
			rotate_forward_in_place(face2);
		else if (face1(1) == face2(0))
			rotate_backward_in_place(face1);
		else if (face1(1) == face2(1))
		{
			rotate_backward_in_place(face1);
			rotate_backward_in_place(face2);
		}
		else if (face1(1) == face2(2))
		{
			rotate_backward_in_place(face1);
			rotate_forward_in_place(face2);
		}
		else
			throw std::exception("same_adjacent_winding_order(): faces are not adjacent");
	}
	// now forced: a * * | a * *
	if (face1(1) == face2(1))
	{
		// a b * | a b *
		return false;
	}
	else if (face1(1) == face2(2))
	{
		// a b * | a * b
		return true;
	}
	else if (face1(2) == face2(1))
	{
		// a * b | a b *
		return true;
	}
	else
	{
		// a * b | a * b
		return false;
	}
}

void standard_deviation(const Eigen::VectorXd& vec, double& mean, double& stddev)
{
	mean = vec.mean();
	stddev = 0.0;
	if (vec.rows() > 0)
		stddev = std::sqrt((vec.array() - mean).square().sum() / vec.rows());
}

bool on_edge(Eigen::RowVector3d query, Eigen::RowVector3d v1, Eigen::RowVector3d v2)
{
	Eigen::RowVector3d dv = v2 - v1;
	Eigen::RowVector3d vq = query - v1;
	if (!dv.cross(vq).isZero())
		return false;
	double dot = dv.dot(vq);
	return 0 <= dot && dot <= dv.dot(dv);
}

bool on_triangle(Eigen::RowVector3d query, Eigen::RowVector3d v1, Eigen::RowVector3d v2, Eigen::RowVector3d v3)
{
	Eigen::RowVector3d v12 = v2 - v1;
	Eigen::RowVector3d v13 = v3 - v1;
	Eigen::RowVector3d vq = query - v1;

	// cramer's rule
	double d00 = v12.dot(v12);
	double d01 = v12.dot(v13);
	double d11 = v13.dot(v13);
	double d20 = vq.dot(v12);
	double d21 = vq.dot(v13);
	double denom = d00 * d11 - d01 * d01;
	if (denom == 0.0)
		return false;
	// barycentric coordinates
	double p = (d11 * d20 - d01 * d21) / denom;
	double q = (d00 * d21 - d01 * d20) / denom;
	double w = 1.0 - p - q;

	return p >= 0.0 && q >= 0.0 && w >= 0.0;
}

bool on_triangle_boundary(Eigen::RowVector3d query, Eigen::RowVector3d v1, Eigen::RowVector3d v2, Eigen::RowVector3d v3)
{
	return on_edge(query, v1, v2) || on_edge(query, v1, v3) || on_edge(query, v2, v3);
}

bool projects_onto_triangle(Eigen::RowVector3d query, Eigen::RowVector3d root, Eigen::RowVector3d prev, Eigen::RowVector3d next)
{
	Eigen::RowVector3d plane_normal = (next - root).cross(prev - root);
	Eigen::RowVector3d projected_point = query - ((query - root).dot(plane_normal) / plane_normal.dot(plane_normal)) * plane_normal;
	return on_triangle(projected_point, root, prev, next);
}

int pos_mod(int pos, size_t mod)
{
	pos = pos % (int)mod;
	return pos >= 0 ? pos : pos + mod;
}

void remove_rows(Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted)
{
	if (indices.empty())
		return;
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

void remove_rows_setup(const Eigen::MatrixXi& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index, const Eigen::RowVector3i&)>& remove_row_predicate)
{
	indices.clear();
	maximum_block_height = 0;
	for (Eigen::Index i = 0; i < mat.rows(); ++i)
	{
		if (remove_row_predicate(i, mat.row(i)))
		{
			if (indices.empty())
				maximum_block_height = i;
			else
				maximum_block_height = std::max(maximum_block_height, i - indices.back() - 1);
			indices.push_back(i);
		}
	}
	if (indices.empty())
		maximum_block_height = mat.rows();
	else
		maximum_block_height = std::max(maximum_block_height, mat.rows() - indices.back() - 1);
}

void remove_rows(Eigen::MatrixXi& mat, const std::function<bool(Eigen::Index, const Eigen::RowVector3i&)>& remove_row_predicate)
{
	std::vector<Eigen::Index> indices;
	Eigen::Index maximum_block_height;
	remove_rows_setup(mat, indices, maximum_block_height, remove_row_predicate);
	remove_rows(mat, indices, maximum_block_height, true);
}

void remove_rows(Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index maximum_block_height, bool indices_sorted)
{
	if (indices.empty())
		return;
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

void remove_rows_setup(const Eigen::MatrixXd& mat, std::vector<Eigen::Index>& indices, Eigen::Index& maximum_block_height, const std::function<bool(Eigen::Index, const Eigen::RowVector3d&)>& remove_row_predicate)
{
	indices.clear();
	maximum_block_height = 0;
	for (Eigen::Index i = 0; i < mat.rows(); ++i)
	{
		if (remove_row_predicate(i, mat.row(i)))
		{
			if (indices.empty())
				maximum_block_height = i;
			else
				maximum_block_height = std::max(maximum_block_height, i - indices.back() - 1);
			indices.push_back(i);
		}
	}
	if (indices.empty())
		maximum_block_height = mat.rows();
	else
		maximum_block_height = std::max(maximum_block_height, mat.rows() - indices.back() - 1);
}

void remove_rows(Eigen::MatrixXd& mat, const std::function<bool(Eigen::Index, const Eigen::RowVector3d&)>& remove_row_predicate)
{
	std::vector<Eigen::Index> indices;
	Eigen::Index maximum_block_height;
	remove_rows_setup(mat, indices, maximum_block_height, remove_row_predicate);
	remove_rows(mat, indices, maximum_block_height, true);
}

void reindex_faces(Eigen::MatrixXi& faces, const std::vector<Eigen::Index>& removed_vertices)
{
	if (removed_vertices.empty())
		return;
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
