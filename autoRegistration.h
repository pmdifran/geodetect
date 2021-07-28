#pragma once
#include "GeoDetection.h"
#include <vector>

Eigen::Matrix4f globalRegistration(GeoDetection& reference, GeoDetection& source, const float& radius = 1.0, const float& subres = 0.15);

//Eigen::Matrix4f icpRegistration(GeoDetection& reference, GeoDetection& source, const float& radus = 1);