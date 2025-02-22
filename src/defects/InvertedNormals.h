#pragma once

#include "Common.h"

namespace defects
{
	struct InvertedNormals : public DefectBase
	{
	protected:
		virtual void _detect(const MeshData& mesh) override;
		virtual void _repair(MeshData& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

		void flip(MeshData& mesh);

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
		void flip(MeshData& mesh, Eigen::Index submesh_root);
	};
}
