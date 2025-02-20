#pragma once

#include "Common.h"
#include "../EquivalenceClasses.h"

// TODO disclaimer that DegenerateVertexPatch may cause duplicate faces
namespace defects
{
	struct DegenerateVertexPatch
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();
		bool in_detected_state() const;

	private:
		EquivalenceClasses vertex_clusters;

	public:
		double threshold = 0.0; // must be positive
	};
}
