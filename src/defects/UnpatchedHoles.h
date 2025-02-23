#pragma once

#include "Common.h"

namespace defects
{
	struct UnpatchedHoles : public DefectBase
	{
	protected:
		virtual void _detect(const MeshData& mesh) override;
		virtual void _repair(MeshData& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		std::vector<std::vector<Eigen::Index>> boundary_vertices;
		std::vector<Eigen::RowVector3i> add_faces;
		std::vector<Eigen::RowVector3d> add_vertices;

		void repair_fan(MeshData& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_strip(MeshData& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_clip(MeshData& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_pie(MeshData& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);

	public:
		enum class PatchMethod
		{
			FAN,
			STRIP,
			CLIP,
			PIE
		} patch_method = PatchMethod::STRIP;
	};
}
