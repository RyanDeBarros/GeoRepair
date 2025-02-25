#pragma once

#include "Common.h"

namespace defects
{
	struct NullFaces : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;
	
	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		std::vector<Eigen::Index> null_face_indices;
		Eigen::Index maximum_block_height = 0;

	public:
		const decltype(null_face_indices)& get_null_face_indices() const { return null_face_indices; }
	};
}
