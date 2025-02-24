#pragma once

#include "Common.h"

namespace defects
{
	struct UnboundVertices : public DefectBase
	{
	protected:
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

		double min_x = -1000.0, max_x = 1000.0;
		double min_y = -1000.0, max_y = 1000.0;
		double min_z = -1000.0, max_z = 1000.0;

	private:
		std::vector<Eigen::Index> unbound_vertices;
	};
}
