#pragma once
#include "GeoDetection.h"
#include <vector>

Eigen::Matrix4f globalRegistration(GeoDetection::Cloud& reference, 
	GeoDetection::Cloud& source, const float& radius = 1.0, const float& subres = 0.15);

Eigen::Matrix4f icpRegistration(GeoDetection::Cloud& reference, 
	GeoDetection::Cloud& source, const float radius = 1);