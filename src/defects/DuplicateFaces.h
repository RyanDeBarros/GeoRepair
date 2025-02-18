#pragma once

#include "Common.h"

namespace defects
{
	struct DuplicateFaces
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();

	private:
		std::vector<Eigen::Index> duplicate_face_indices;
		Eigen::Index maximum_block_height = 0;
	};
}
