#pragma once
#include "GeoDetection.h"
#include "registration_cloud.h"
#include <vector>

namespace GeoDetection
{
	Eigen::Matrix4f getGlobalRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, float radius = 1.0, float subres = 0.15);

	Eigen::Matrix4f getGlobalRegistration(GeoDetection::RegistrationCloud& reference,
		GeoDetection::Cloud& source, float radius = 1.0, float subres = 0.15);

	Eigen::Matrix4f getICPRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, float radius = 1);
}