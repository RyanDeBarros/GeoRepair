#include "NoiseSmoothing.h"

// TODO test all detection and smoothing methods

void defects::NoiseSmoothing::_detect(const Mesh& mesh)
{
	void(defects::NoiseSmoothing::*detect_func)(const Mesh&) = nullptr;
	switch (detection_method)
	{
	case DetectionMethod::LAPLACIAN_RESIDUAL:
		detect_func = &defects::NoiseSmoothing::detect_laplacian_residual;
		break;
	case DetectionMethod::MEAN_CURVATURE:
		detect_func = &defects::NoiseSmoothing::detect_mean_curvature;
		break;
	case DetectionMethod::FEATURE_SENSITIVE:
		detect_func = &defects::NoiseSmoothing::detect_feature_sensitive;
		break;
	case DetectionMethod::EIGEN_VALUES:
		detect_func = &defects::NoiseSmoothing::detect_eigen_values;
		break;
	}

	(this->*detect_func)(mesh);
}

void defects::NoiseSmoothing::_repair(Mesh& mesh)
{
	void (defects::NoiseSmoothing::*repair_func)(Mesh&) = nullptr;
	switch (smoothing_method)
	{
	case SmoothingMethod::LAPLACIAN:
		repair_func = &defects::NoiseSmoothing::repair_laplacian;
		break;
	case SmoothingMethod::TAUBIN:
		repair_func = &defects::NoiseSmoothing::repair_taubin;
		break;
	case SmoothingMethod::DESBRUN:
		repair_func = &defects::NoiseSmoothing::repair_desbrun;
		break;
	case SmoothingMethod::BILATERAL:
		repair_func = &defects::NoiseSmoothing::repair_bilateral;
		break;
	}

	(this->*repair_func)(mesh);
}

void defects::NoiseSmoothing::reset()
{
	noisy_vertices.clear();
}

bool defects::NoiseSmoothing::in_detected_state() const
{
	return !noisy_vertices.empty();
}

void defects::NoiseSmoothing::detect_laplacian_residual(const Mesh& mesh)
{
	const auto& residuals = mesh.get_laplacian_residuals();
	double mean, stddev;
	standard_deviation(residuals, mean, stddev);
	double threshold = mean + stddev * laplacian_tolerance; // if residue exceeds threshold, then statistically it is an outlier and therefore noisy.
	for (Eigen::Index i = 0; i < residuals.rows(); ++i)
	{
		if (residuals(i) > threshold)
			noisy_vertices.push_back(i);
	}
}

void defects::NoiseSmoothing::detect_mean_curvature(const Mesh& mesh)
{
	const auto& magnitudes = mesh.get_mean_curvature_magnitudes();
	double mean, stddev;
	standard_deviation(magnitudes, mean, stddev);
	double threshold = mean + stddev * curvature_tolerance; // if magnitude exceeds threshold, then statistically it is an outlier and therefore noisy.
	for (Eigen::Index i = 0; i < magnitudes.rows(); ++i)
	{
		if (magnitudes(i) > threshold)
			noisy_vertices.push_back(i);
	}
}

void defects::NoiseSmoothing::detect_feature_sensitive(const Mesh& mesh)
{
	double mean, stddev;
	const auto& residuals = mesh.get_laplacian_residuals();
	standard_deviation(residuals, mean, stddev);
	double laplacian_threshold = mean + stddev * laplacian_tolerance;
	const auto& magnitudes = mesh.get_mean_curvature_magnitudes();
	standard_deviation(magnitudes, mean, stddev);
	double curvature_threshold = mean + stddev * curvature_tolerance;

	for (Eigen::Index i = 0; i < residuals.rows(); ++i)
	{
		// If residual is high:
		//     - if magnitude is low: likely noise
		//     - if magnitude is high: likely feature
		// If residual is low:
		//     - if magnitude is low: likely no noise
		//     - if magnitude is high: likely sharp feature
		if (residuals(i) > laplacian_threshold && magnitudes(i) <= curvature_threshold)
			noisy_vertices.push_back(i);
	}
}

void defects::NoiseSmoothing::detect_eigen_values(const Mesh& mesh)
{
	eigen_decomposition_largest(mesh.get_laplacian(), eigen_count, eigen_vectors, eigen_values);
	for (Eigen::Index i = 0; i < eigen_count; ++i)
	{
		if (eigen_values[i] > eigen_value_threshold)
			noisy_vertices.push_back(i);
	}
}

static void laplacian_smoothing(Eigen::MatrixXd& vertices, const std::vector<Eigen::Index> indices, const Eigen::MatrixXd& laplacian_eval, double factor)
{
	for (Eigen::Index i : indices)
	{
		Eigen::RowVector3d v = vertices.row(i);
		vertices.row(i) = v - factor * laplacian_eval.row(i);
	}
}

static void laplacian_smoothing(Eigen::MatrixXd& vertices, const std::vector<Eigen::Index> indices, const Eigen::SparseMatrix<double>& laplacian, double factor)
{
	for (Eigen::Index i : indices)
	{
		Eigen::RowVector3d laplacian_eval = laplacian.row(i) * vertices;
		Eigen::RowVector3d v = vertices.row(i);
		vertices.row(i) = v - factor * laplacian_eval;
	}
}

void defects::NoiseSmoothing::repair_laplacian(Mesh& mesh)
{
	auto& vertices = mesh.get_vertices();
	laplacian_smoothing(vertices, noisy_vertices, mesh.get_laplacian_eval(), laplacian_smoothing_factor);
	if (laplacian_iterations > 1)
	{
		const auto& laplacian = mesh.get_laplacian();
		for (int i = 1; i < laplacian_iterations; ++i)
			laplacian_smoothing(vertices, noisy_vertices, laplacian, laplacian_smoothing_factor);
	}
}

void defects::NoiseSmoothing::repair_taubin(Mesh& mesh)
{
	auto& vertices = mesh.get_vertices();
	laplacian_smoothing(vertices, noisy_vertices, mesh.get_laplacian_eval(), laplacian_smoothing_factor);
	const auto& laplacian = mesh.get_laplacian();
	laplacian_smoothing(vertices, noisy_vertices, laplacian, -taubin_counter_smoothing_factor);
	for (int i = 1; i < laplacian_iterations; ++i)
	{
		laplacian_smoothing(vertices, noisy_vertices, laplacian, laplacian_smoothing_factor);
		laplacian_smoothing(vertices, noisy_vertices, laplacian, -taubin_counter_smoothing_factor);
	}
}

static void desbrun_smoothing(Eigen::MatrixXd& vertices, const std::vector<Eigen::Index> indices, const Eigen::MatrixXd& laplacian_eval, const Eigen::MatrixXd& normals, double factor)
{
	for (Eigen::Index i : indices)
	{
		Eigen::RowVector3d v = vertices.row(i);
		vertices.row(i) = v - factor * (laplacian_eval.row(i) - laplacian_eval.row(i).dot(normals.row(i)) * normals.row(i));
	}
}

static void desbrun_smoothing(Eigen::MatrixXd& vertices, const std::vector<Eigen::Index> indices, const Eigen::SparseMatrix<double>& laplacian, const Eigen::MatrixXd& normals, double factor)
{
	for (Eigen::Index i : indices)
	{
		Eigen::RowVector3d laplacian_eval = laplacian.row(i) * vertices;
		Eigen::RowVector3d v = vertices.row(i);
		vertices.row(i) = v - factor * (laplacian_eval - laplacian_eval.dot(normals.row(i)) * normals.row(i));
	}
}

void defects::NoiseSmoothing::repair_desbrun(Mesh& mesh)
{
	auto& vertices = mesh.get_vertices();
	desbrun_smoothing(vertices, noisy_vertices, mesh.get_laplacian_eval(), mesh.get_vertex_normals(), laplacian_smoothing_factor);
	if (laplacian_iterations > 1)
	{
		const auto& laplacian = mesh.get_laplacian();
		for (int i = 1; i < laplacian_iterations; ++i)
		{
			Eigen::MatrixXd normals;
			igl::per_vertex_normals(vertices, mesh.get_faces(), normals); // TODO later only re-compute normals for noisy vertices
			desbrun_smoothing(vertices, noisy_vertices, laplacian, normals, laplacian_smoothing_factor);
		}
	}
}

void defects::NoiseSmoothing::repair_bilateral(Mesh& mesh)
{
	auto& vertices = mesh.get_vertices();
	const auto& normals = mesh.get_vertex_normals();
	std::vector<Eigen::RowVector3d> new_vertices;
	new_vertices.resize(noisy_vertices.size());
	for (size_t i = 0; i < noisy_vertices.size(); ++i)
	{
		Eigen::Index v = noisy_vertices[i];
		const auto& neighbourhood = mesh.get_adj_vertices(v);
		double sum = 0.0, normalizer = 0.0;
		Eigen::RowVector3d vertex = vertices.row(v);
		Eigen::RowVector3d normal = normals.row(v);
		for (Eigen::Index neighbour : neighbourhood)
		{
			Eigen::RowVector3d displacement = vertex - vertices.row(neighbour);
			double t = displacement.norm();
			double h = normal.dot(displacement);
			double w_c = std::exp(-t * t / (2 * bilateral_closeness_factor * bilateral_closeness_factor));
			double w_s = std::exp(-h * h / (2 * bilateral_similarity_factor * bilateral_similarity_factor));
			double w = w_c * w_s;
			sum += w * h;
			normalizer += w;
		}
		new_vertices[i] = vertex + bilateral_smoothing_factor * normal * (sum / normalizer);
	}
	for (size_t i = 0; i < noisy_vertices.size(); ++i)
		vertices.row(noisy_vertices[i]) = new_vertices[i];
}
