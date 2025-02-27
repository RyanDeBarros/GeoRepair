#pragma once

#include "Common.h"

namespace defects
{
	struct NoiseSmoothing : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

		enum class DetectionMethod
		{
			LAPLACIAN_RESIDUAL,
			MEAN_CURVATURE,
			FEATURE_SENSITIVE // combines laplacian residual and mean curvature
		} detection_method = DetectionMethod::FEATURE_SENSITIVE;

		bool smooth_all = false;
		bool ignore_boundaries = true;
		bool global_noise = true; // TODO implement local noise
		double laplacian_tolerance = 1.0; // must be positive - only used for LAPLACIAN_RESIDUAL and FEATURE_SENSITIVE
		double curvature_tolerance = 1.0; // must be positive - only used for MEAN_CURVATURE and FEATURE_SENSITIVE

		enum class SmoothingMethod
		{
			LAPLACIAN,
			TAUBIN,
			DESBRUN,
			BILATERAL
		} smoothing_method = SmoothingMethod::TAUBIN;

		int laplacian_iterations = 3; // must be > 0 - only used for LAPLACIAN
		float laplacian_smoothing_factor = 0.4f; // must be between 0.0 and 1.0 - only used for LAPLACIAN
		int taubin_iterations = 3; // must be > 0 - only used for TAUBIN
		float taubin_shrinking_factor = 0.6f; // must be between 0.0 and 1.0 - only used for TAUBIN
		float taubin_expanding_factor = -0.3f; // must be between -1.0 and 0.0 - only used for TAUBIN
		int desbrun_iterations = 3; // must be > 0 - only used for DESBRUN
		float desbrun_smoothing_factor = 0.3f; // must be between 0.0 and 1.0 - only used for DESBRUN
		float bilateral_tangential_factor = 1.0f; // must be > 0.0 - only used for BILATERAL
		float bilateral_normal_factor = 1.0f; // must be > 0.0 - only used for BILATERAL
		float bilateral_smoothing_factor = 0.5f; // must be between 0.0 and 1.0 - only used for BILATERAL

	private:
		void detect_laplacian_residual(const Mesh& mesh);
		void detect_mean_curvature(const Mesh& mesh);
		void detect_feature_sensitive(const Mesh& mesh);

		void repair_laplacian(Mesh& mesh);
		void repair_taubin(Mesh& mesh);
		void repair_desbrun(Mesh& mesh);
		void repair_bilateral(Mesh& mesh);

		std::vector<Eigen::Index> noisy_vertices;
		Eigen::VectorXd eigen_values;

	public:
		const decltype(noisy_vertices)& get_noisy_vertices() const { return noisy_vertices; }
	};
}
