#pragma once

#include "Common.h"

namespace defects
{
	struct DuplicateFaces
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();
		bool in_detected_state() const;

	private:
		std::vector<Eigen::Index> duplicate_face_indices;
		Eigen::Index maximum_block_height = 0;

	public:
		bool ignore_normals = false;

		const decltype(duplicate_face_indices)& get_duplicate_face_indices() const { return duplicate_face_indices; }
	};
}
