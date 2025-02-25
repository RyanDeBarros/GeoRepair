#pragma once

#include "DegenerateVertexPatch.h"
#include "DuplicateFaces.h"

namespace defects
{
	// combines DegenerateVertexPatch and DuplicateFaces
	struct DegenerateFaces : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		DegenerateVertexPatch degenerate_vertex_patch;
		DuplicateFaces duplicate_faces;

	public:
		double tolerance = 0.0; // must be positive
		bool ignore_normals = false;

		const DegenerateVertexPatch& get_degenerate_vertex_patch() const { return degenerate_vertex_patch; }
		const DuplicateFaces& get_duplicate_faces() const { return duplicate_faces; }
	};
}
