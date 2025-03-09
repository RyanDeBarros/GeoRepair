#pragma once

#include "Common.h"

namespace defects
{
	struct UnconnectedSubmeshes : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		std::vector<std::vector<Eigen::Index>> submeshes;

	public:
		const decltype(submeshes)& get_submeshes() const { return submeshes; }
	};
}
