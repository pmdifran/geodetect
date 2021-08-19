#pragma once
#include "GeoDetection.h"
#include <vector>

namespace GeoDetection
{
	Eigen::Matrix4f getGlobalRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, const float& radius = 1.0, const float& subres = 0.15);

	Eigen::Matrix4f getICPRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, const float radius = 1);
}