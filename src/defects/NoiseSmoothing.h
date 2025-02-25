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
			FEATURE_SENSITIVE, // combines laplacian residual and mean curvature
			EIGEN_VALUES // expensive
		} detection_method = DetectionMethod::FEATURE_SENSITIVE;

		double laplacian_tolerance = 1.0; // must be positive - only used for LAPLACIAN_RESIDUAL and FEATURE_SENSITIVE
		double curvature_tolerance = 1.0; // must be positive - only used for MEAN_CURVATURE and FEATURE_SENSITIVE
		int eigen_count = 10; // must be positive - only used for EIGEN_VALUES
		double eigen_value_threshold = 1.0; // must be positive - only used for EIGEN_VALUES

		enum class SmoothingMethod
		{
			LAPLACIAN,
			TAUBIN,
			DESBRUN,
			BILATERAL
		} smoothing_method = SmoothingMethod::TAUBIN;

		// TODO consider an option to ignore boundary vertices

		double laplacian_smoothing_factor = 0.6; // must be between 0.0 and 1.0 - only used for LAPLACIAN and TAUBIN
		int laplacian_iterations = 3; // must be > 1 - only used for LAPLACIAN and TAUBIN
		double taubin_counter_smoothing_factor = 0.8; // must be between laplacian_smoothing_factor and 1.0 - only used for TAUBIN
		double desbrun_smoothing_factor = 0.5; // must be between 0.0 and 1.0 - only used for DESBRUN
		double bilateral_closeness_factor = 1.0; // must be > 0.0 - only used for BILATERAL
		double bilateral_similarity_factor = 1.0; // must be > 0.0 - only used for BILATERAL
		double bilateral_smoothing_factor = 0.5; // must be between 0.0 and 1.0 - only used for BILATERAL

	private:
		void detect_laplacian_residual(const Mesh& mesh);
		void detect_mean_curvature(const Mesh& mesh);
		void detect_feature_sensitive(const Mesh& mesh);
		void detect_eigen_values(const Mesh& mesh);

		void repair_laplacian(Mesh& mesh);
		void repair_taubin(Mesh& mesh);
		void repair_desbrun(Mesh& mesh);
		void repair_bilateral(Mesh& mesh);

		std::vector<Eigen::Index> noisy_vertices;

		Eigen::MatrixXd eigen_vectors;
		Eigen::VectorXd eigen_values;
	};
}
