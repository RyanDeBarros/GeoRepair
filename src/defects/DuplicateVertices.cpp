#include "DuplicateVertices.h"

void remove_duplicate_vertices(MeshData& mesh, EquivalenceClasses& vertex_clusters)
{
	auto& vertices = mesh.get_vertices();
	auto& faces = mesh.get_faces();
	std::vector<Eigen::Index> remove_vertices, remove_faces;
	std::vector<bool> removed_faces_check(faces.rows(), false);
	auto clusters = vertex_clusters.gen_classes();

	for (const auto& [root, cluster] : clusters)
	{
		Eigen::RowVector3d mean(0.0, 0.0, 0.0);
		for (auto iter = cluster.begin(); iter != cluster.end(); ++iter)
		{
			Eigen::Index v = *iter;
			mean += vertices.row(v);
			if (v != root)
			{
				// Mark vertex for removal
				remove_vertices.push_back(v);
				const auto& adj_face_rows = mesh.get_adj_face_rows(v);
				const auto& adj_face_cols = mesh.get_adj_face_cols(v);
				for (Eigen::Index i = 0; i < adj_face_rows.size(); ++i)
				{
					Eigen::Index fi = adj_face_rows[i];
					if (!removed_faces_check[fi])
					{
						bool mark_face = false;
						Eigen::Index col = adj_face_cols[i];
						for (int c = 0; c < 3; ++c)
						{
							if (c != col && cluster.count(faces(fi, c)))
								// Degenerate face caused by cluster collapse. Mark for removal
								mark_face = true;
						}
						if (mark_face)
						{
							remove_faces.push_back(fi);
							removed_faces_check[fi] = true;
						}
						else
							// Replace duplicate vertices with root index
							faces(fi, col) = root;
					}
				}
			}
		}
		mean *= 1.0 / cluster.size();
		// Replace root with mean point
		vertices.row(root) = mean;
	}
	// Remove marked vertices
	remove_rows(vertices, remove_vertices, false);
	// Remove marked faces
	remove_rows(faces, remove_faces, false);
	// Re-index faces
	reindex_faces(faces, remove_vertices);
}
