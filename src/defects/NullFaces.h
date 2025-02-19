#pragma once

#include "Common.h"

namespace defects
{
	struct NullFaces
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();
		bool in_detected_state() const;

	private:
		std::vector<Eigen::Index> null_face_indices;
		Eigen::Index maximum_block_height = 0;

	public:
		const decltype(null_face_indices)& get_null_face_indices() const { return null_face_indices; }
	};
}
