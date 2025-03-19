#pragma once

#include "Common.h"

namespace defects
{
	struct IsolatedVertices : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		std::vector<Eigen::Index> isolated_vertices;

	public:
		const decltype(isolated_vertices)& get_isolated_vertices() const { return isolated_vertices; }
	};
}
