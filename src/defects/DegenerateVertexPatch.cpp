#include "DegenerateVertexPatch.h"

// TODO use spatial hashing instead of neighbourhood for general distanced vertices. no logical way of combining the vertices without self-intersections, though, but still maybe useful for other algorithms. Maybe a general-space vertex distance detector that doesn't repair anything.
void defects::DegenerateVertexPatch::detect(const MeshData& mesh)
{
	reset();
	double sqrd_acceptance = acceptance * acceptance;
	const auto& vertices = mesh.get_vertices();
	std::unordered_set<Eigen::Index> already_processed;
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		already_processed.insert(i);
		Eigen::RowVector3d vi = vertices.row(i);
		const auto& neighbourhood = mesh.get_adj_vertices(i);
		for (Eigen::Index j : neighbourhood)
		{
			if (!already_processed.count(j) && (vertices.row(j) - vi).squaredNorm() <= sqrd_acceptance)
				vertex_clusters.add(i, j);
		}
	}
}

void defects::DegenerateVertexPatch::repair(MeshData& mesh)
{
	if (in_detected_state())
	{
		auto& vertices = mesh.get_vertices();
		auto& faces = mesh.get_faces();
		std::vector<Eigen::Index> remove_vertices, remove_faces;
		std::unordered_set<Eigen::Index> removed_faces_check;
		auto classes = vertex_clusters.gen_classes();

		for (const auto& [root, cluster] : classes)
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
						if (!removed_faces_check.count(fi))
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
								removed_faces_check.insert(fi);
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
		reset();
	}
}

void defects::DegenerateVertexPatch::reset()
{
	vertex_clusters.clear();
}

bool defects::DegenerateVertexPatch::in_detected_state() const
{
	return !vertex_clusters.empty();
}
