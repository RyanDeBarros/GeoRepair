#pragma once

#include "Common.h"

// TODO disclaimer that DegenerateVertexPatch may cause duplicate faces
namespace defects
{
	struct DegenerateVertexPatch : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		EquivalenceClasses vertex_clusters;

	public:
		double tolerance = 0.0; // must be positive

		const EquivalenceClasses& get_vertex_clusters() const { return vertex_clusters; }
	};
}
