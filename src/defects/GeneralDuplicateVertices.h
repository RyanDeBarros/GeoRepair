#pragma once

#include "Common.h"

namespace defects
{
	struct GeneralDuplicateVertices
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
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
		double tolerance = 0.0; // must be positive
	};
}
