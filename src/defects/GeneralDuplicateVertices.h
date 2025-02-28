#pragma once

#include "Common.h"

namespace defects
{
	struct GeneralDuplicateVertices : public DefectBase
	{
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
		void detect_exact(const Mesh& mesh);

		std::vector<Proximity> proximities;
		std::unordered_map<Eigen::Index, double> squared_distances;

	public:
		double tolerance = 0.0; // must be positive

		const decltype(squared_distances)& get_squared_distances() const { return squared_distances; }
	};
}
