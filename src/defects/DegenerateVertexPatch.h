#pragma once

#include "Common.h"

// TODO disclaimer that DegenerateVertexPatch may cause duplicate faces
namespace defects
{
	struct DegenerateVertexPatch : public DefectBase
	{
	protected:
		virtual void _detect(const MeshData& mesh) override;
		virtual void _repair(MeshData& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		EquivalenceClasses vertex_clusters;

	public:
		double tolerance = 0.0; // must be positive
	};
}
