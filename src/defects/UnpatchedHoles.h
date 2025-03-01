#pragma once

#include "Common.h"

namespace defects
{
	struct UnpatchedHoles : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		std::vector<std::vector<Eigen::Index>> boundary_vertices;
		std::vector<Eigen::RowVector3i> add_faces;
		std::vector<Eigen::RowVector3d> add_vertices;

		void repair_fan(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_strip(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_clip(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_pie(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);
		void repair_ear_clipping(Mesh& mesh, const std::vector<Eigen::Index>& boundary, bool increasing);

	public:
		enum class PatchMethod
		{
			FAN,
			STRIP,
			CLIP,
			PIE,
			EAR_CLIPPING
		} patch_method = PatchMethod::EAR_CLIPPING;

		const decltype(boundary_vertices)& get_boundary_vertices() const { return boundary_vertices; }
	};
}
