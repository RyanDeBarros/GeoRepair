#pragma once

#include "Common.h"
#include "../EquivalenceClasses.h"

namespace defects
{
	struct DegenerateFaces
	{
		double threshold = 0.0; // must be positive

		bool detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();

	private:
		std::vector<Eigen::Index> degenerate_face_indices;
		EquivalenceClasses vertex_classes;
		Eigen::Index face_maximum_block_height = 0;

	public:
		const decltype(degenerate_face_indices)& get_degenerate_face_indices() const { return degenerate_face_indices; }
	};
}
