#pragma once

#include "Common.h"

namespace defects
{
	struct VertexProximityDetection
	{
		void detect(const MeshData& mesh);
		// no repair(), since logically, collapsing vertices that aren't connected will just cause self-intersections and complications.
		void reset();
		bool in_detected_state() const;

		struct Proximity
		{
			Eigen::Index i, j;
			double dist_squared;
		};

	private:
		std::vector<Proximity> proximities;

	public:
		double threshold = 0.0; // must be positive
	};
}
