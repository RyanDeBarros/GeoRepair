#pragma once

#include "Common.h"

namespace defects
{
	struct DuplicateFaces : public DefectBase
	{
	protected:
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		std::vector<Eigen::Index> duplicate_face_indices;
		Eigen::Index maximum_block_height = 0;

	public:
		bool ignore_normals = false;

		const decltype(duplicate_face_indices)& get_duplicate_face_indices() const { return duplicate_face_indices; }
	};
}
