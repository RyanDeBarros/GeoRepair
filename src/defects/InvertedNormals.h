#pragma once

#include "Common.h"

namespace defects
{
	struct InvertedNormals
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();
		bool in_detected_state() const;

		void flip(MeshData& mesh);

	private:
		enum class WindingAdjustment
		{
			UNPROCESSED,
			KEEP,
			FLIP
		};
		std::vector<WindingAdjustment> winding_adjustments;
		std::vector<std::vector<Eigen::Index>> flip_faces;
		std::vector<Eigen::Index> submesh_roots;
		std::vector<Eigen::Index> flip_entire_submesh_roots;
		void flip(MeshData& mesh, Eigen::Index submesh_root);
	};
}
