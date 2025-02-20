#pragma once

#include "DegenerateVertexPatch.h"
#include "DuplicateFaces.h"

namespace defects
{
	// combines DegenerateVertexPatch and DuplicateFaces
	struct DegenerateFaces
	{
		void detect(const MeshData& mesh);
		void repair(MeshData& mesh);
		void reset();
		bool in_detected_state() const;

	private:
		DegenerateVertexPatch degenerate_vertex_patch;
		DuplicateFaces duplicate_faces;

	public:
		double tolerance = 0.0; // must be positive
		bool ignore_normals = false;
	};
}
