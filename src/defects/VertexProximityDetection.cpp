#include "VertexProximityDetection.h"

void defects::VertexProximityDetection::detect(const MeshData& mesh)
{
	reset();

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
}

void defects::VertexProximityDetection::reset()
{
	proximities.clear();
}

bool defects::VertexProximityDetection::in_detected_state() const
{
	return !proximities.empty();
}
