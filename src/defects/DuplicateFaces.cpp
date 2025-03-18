#include "DuplicateFaces.h"

#include <unordered_set>

struct UndirectedFaceIgnoringNormals
{
	int v1, v2, v3;

	bool operator==(const UndirectedFaceIgnoringNormals& other) const
	{
		return (v1 == other.v1 && v2 == other.v2 && v3 == other.v3)
			|| (v1 == other.v3 && v2 == other.v1 && v3 == other.v2)
			|| (v1 == other.v2 && v2 == other.v3 && v3 == other.v1)
			|| (v1 == other.v1 && v2 == other.v3 && v3 == other.v2)
			|| (v1 == other.v3 && v2 == other.v2 && v3 == other.v1)
			|| (v1 == other.v2 && v2 == other.v1 && v3 == other.v3);
	}
};

struct UndirectedFace
{
	int v1, v2, v3;

	bool operator==(const UndirectedFace& other) const
	{
		return (v1 == other.v1 && v2 == other.v2 && v3 == other.v3)
			|| (v1 == other.v3 && v2 == other.v1 && v3 == other.v2)
			|| (v1 == other.v2 && v2 == other.v3 && v3 == other.v1);
	}
};

struct UndirectedFaceHash
{
	size_t operator()(const UndirectedFaceIgnoringNormals& f) const { return std::hash<int>{}(f.v1) ^ std::hash<int>{}(f.v2) ^ std::hash<int>{}(f.v3); }
	size_t operator()(const UndirectedFace& f) const { return std::hash<int>{}(f.v1) ^ std::hash<int>{}(f.v2) ^ std::hash<int>{}(f.v3); }
};

void defects::DuplicateFaces::_detect(const Mesh& mesh)
{
	const Eigen::MatrixXi& faces = mesh.get_faces();
	if (ignore_normals)
	{
		std::unordered_map<UndirectedFaceIgnoringNormals, std::tuple<int, Eigen::Index>, UndirectedFaceHash> face_counter;
		remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&face_counter](Eigen::Index i, const Eigen::RowVector3i& face) {
			auto f = UndirectedFaceIgnoringNormals{ face(0), face(1), face(2) };
			auto it = face_counter.find(f);
			if (it != face_counter.end())
			{
				++std::get<0>(it->second);
				return true;
			}
			else
			{
				face_counter[f] = { 1, i };
				return false;
			}
			});
		for (Eigen::Index i : duplicate_face_indices)
		{
			const Eigen::RowVector3i& face = faces.row(i);
			auto f = UndirectedFaceIgnoringNormals{ face(0), face(1), face(2) };
			duplicate_face_roots.push_back(std::get<1>(face_counter.find(f)->second));
		}
	}
	else
	{
		std::unordered_map<UndirectedFace, std::tuple<int, Eigen::Index>, UndirectedFaceHash> face_counter;
		remove_rows_setup(faces, duplicate_face_indices, maximum_block_height, [&face_counter](Eigen::Index i, const Eigen::RowVector3i& face) {
			auto f = UndirectedFace{ face(0), face(1), face(2) };
			auto it = face_counter.find(f);
			if (it != face_counter.end())
			{
				++std::get<0>(it->second);
				return true;
			}
			else
			{
				face_counter[f] = { 1, i };
				return false;
			}
			});
		for (Eigen::Index i : duplicate_face_indices)
		{
			const Eigen::RowVector3i& face = faces.row(i);
			auto f = UndirectedFace{ face(0), face(1), face(2) };
			duplicate_face_roots.push_back(std::get<1>(face_counter.find(f)->second));
		}
	}
}

void defects::DuplicateFaces::_repair(Mesh& mesh)
{
	remove_rows(mesh.get_faces(), duplicate_face_indices, maximum_block_height, true);
	mesh.reset_face_colors();
}

void defects::DuplicateFaces::reset()
{
	duplicate_face_indices.clear();
	duplicate_face_roots.clear();
	maximum_block_height = 0;
}

bool defects::DuplicateFaces::in_detected_state() const
{
	return !duplicate_face_indices.empty();
}
