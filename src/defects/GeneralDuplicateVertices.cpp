#include "GeneralDuplicateVertices.h"

#include "DuplicateVertices.h"

void defects::GeneralDuplicateVertices::_detect(const Mesh& mesh)
{
	std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Index>, EigenMatrixHash> voxel_grid;
	const double inv_tolerance = 1.0 / tolerance;
	const double sqrd_tolerance = tolerance * tolerance;
	const auto& vertices = mesh.get_vertices();

	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		Eigen::RowVector3d vi = vertices.row(i);
		// quantized voxel
		Eigen::Vector3i voxel = (vi * inv_tolerance).array().floor().cast<int>();
		// check neighbouring voxels
		for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) for (int dz = -1; dz <= 1; ++dz)
		{
			Eigen::Vector3i neighbour_voxel = voxel + Eigen::Vector3i(dx, dy, dz);
			auto it = voxel_grid.find(neighbour_voxel);
			if (it != voxel_grid.end())
			{
				for (Eigen::Index j : it->second)
				{
					double sqrd_distance = (vi - vertices.row(j)).squaredNorm();
					if (sqrd_distance <= sqrd_tolerance)
						proximities.emplace_back(Proximity{ i, j, sqrd_distance });
				}
			}
		}
		voxel_grid[voxel].push_back(i);
	}

	std::unordered_map<Eigen::Index, int> number_of_neighbours;
	std::unordered_map<Eigen::Index, double> squared_distance_sum;
	for (const Proximity& p : proximities)
	{
		++number_of_neighbours[p.i];
		++number_of_neighbours[p.j];
		squared_distance_sum[p.i] += p.dist_squared;
		squared_distance_sum[p.j] += p.dist_squared;
	}
	for (const auto& [v, num_neighbours] : number_of_neighbours)
		squared_distances[v] = squared_distance_sum[v] / num_neighbours;
}

void defects::GeneralDuplicateVertices::_repair(Mesh& mesh)
{
	EquivalenceClasses vertex_classes;
	for (const auto& proximity : proximities)
		vertex_classes.add(proximity.i, proximity.j);

	remove_duplicate_vertices(mesh, vertex_classes);
}

void defects::GeneralDuplicateVertices::reset()
{
	proximities.clear();
	squared_distances.clear();
}

bool defects::GeneralDuplicateVertices::in_detected_state() const
{
	return !proximities.empty();
}
