#pragma once
#include "GeoDetection.h"
#include "registration_cloud.h"
#include <vector>

namespace GeoDetection
{
	/**
	* Transforms source cloud using a global registration determined with keypoint fast point feature histogram correspondences.
	* @param[in] reference: target GeoDetection::Cloud (i.e. which we are aligning to)
	* @param[out] source: GeoDetection::Cloud that we are aligning
	* @param[in] radius: normal radius to compute features at. Default = 1.0. Not used if the cloud already has normals.
	*/
	Eigen::Matrix4f getGlobalRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, float radius = 1.0);

	/**
	* Transforms source cloud using a global registration determined with keypoints' fast point feature histogram correspondences.
	* Registration cloud is used in this overload, which reuses keypoints and their computed fpfh's, if they have been computed
	* @param[in] reference: target GeoDetection::RegistrationCloud (i.e. which we are aligning to)
	* @param[out] source: GeoDetection::Cloud that we are aligning
	* @param[in] radius: normal radius to compute features at. Default = 1.0. Not used if the cloud already has normals.
	*/
	Eigen::Matrix4f getGlobalRegistration(GeoDetection::RegistrationCloud& reference,
		GeoDetection::Cloud& source, float radius = 1.0);

	/**
	* Transforms source cloud using a fine generalized ICP registration (i.e. plane-to-plane).
	* @param[in] reference: target GeoDetection::Cloud (i.e. which we are aligning to)
	* @param[out] source: GeoDetection::Cloud that we are aligning
	* @param[in] radius: normal radius to compute features at. Default = 1.0. Not used if the cloud already has normals.
	*/
	Eigen::Matrix4f getICPRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, float radius = 1.0);
}