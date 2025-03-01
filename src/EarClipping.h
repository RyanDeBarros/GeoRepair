#pragma once

#include <Eigen/core>

extern void ear_clipping(const std::vector<Eigen::Index>& boundary, const Eigen::MatrixXd& vertices, std::vector<Eigen::RowVector3i>& add_faces, bool increasing, int ear_cycle, int reference_point_offset);
