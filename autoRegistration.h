#pragma once
#include "GeoDetection.h"

Eigen::Matrix4f globalRegistration(GeoDetection& reference, GeoDetection& source, const float& radius = 1, const float& subres = 0.15);

//Eigen::Matrix4f icpRegistration(GeoDetection& reference, GeoDetection& source, const float& radus = 1);