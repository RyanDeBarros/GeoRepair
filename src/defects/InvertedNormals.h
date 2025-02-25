#pragma once

#include "Common.h"

namespace defects
{
	struct InvertedNormals : public DefectBase
	{
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

	public:
		const decltype(flip_faces)& get_flip_faces() const { return flip_faces; }
		const decltype(submesh_roots)& get_submesh_roots() const { return submesh_roots; }
		const decltype(flip_entire_submesh_roots)& get_flip_entire_submesh_roots() const { return flip_entire_submesh_roots; }
	};
}
