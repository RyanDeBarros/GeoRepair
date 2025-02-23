#pragma once

#include "Common.h"

namespace defects
{
	struct InvertedNormals : public DefectBase
	{
	protected:
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

		void flip(Mesh& mesh);

	private:
		enum class WindingAdjustment
		{
			UNPROCESSED,
			KEEP,
			FLIP
		};
		std::vector<WindingAdjustment> winding_adjustments;
		std::vector<std::vector<Eigen::Index>> flip_faces;
		std::vector<Eigen::Index> submesh_roots;
		std::vector<Eigen::Index> flip_entire_submesh_roots;
		void flip(Mesh& mesh, Eigen::Index submesh_root);
	};
}
