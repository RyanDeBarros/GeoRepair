#pragma once

#include <Eigen/core>
#include <imgui.h>

namespace colors
{
	namespace style
	{
		extern ImVec4 detected;
	}
	namespace vertex
	{
		extern Eigen::RowVector3d neutral;
		extern Eigen::RowVector3d defective;
		extern Eigen::RowVector3d defective_low;
	}
	namespace face
	{
		extern Eigen::RowVector3d neutral;
		extern Eigen::RowVector3d defective;
		extern Eigen::RowVector3d defective_alt;
	}
	namespace edge
	{
		extern Eigen::RowVector3d defective;
		extern Eigen::RowVector3d defective_alt;
	}
}
