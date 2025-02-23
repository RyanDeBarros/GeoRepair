#pragma once

#include "Common.h"

namespace defects
{
	struct GeneralDuplicateVertices : public DefectBase
	{
	protected:
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

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
