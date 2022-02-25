#pragma once
#include "geodetect.h"
#include "registration_cloud.h"
#include <vector>

namespace geodetect
{
	/**
	* Transforms source cloud using a global registration determined with keypoint fast point feature histogram correspondences.
	* @param[in] reference: target geodetect::Cloud (i.e. which we are aligning to)
	* @param[out] source: geodetect::Cloud that we are aligning
	* @param[in] radius: normal radius to compute features at. Default = 1.0. Not used if the cloud already has normals.
	*/
	Eigen::Matrix4f getGlobalRegistration(geodetect::Cloud& reference,
		geodetect::Cloud& source, float normal_scale, float scale_coefficient);

	/**
	* Transforms source cloud using a fine generalized ICP registration (i.e. plane-to-plane).
	* @param[in] reference: target geodetect::Cloud (i.e. which we are aligning to)
	* @param[out] source: geodetect::Cloud that we are aligning
	* @param[in] radius: normal radius to compute features at. Default = 1.0. Not used if the cloud already has normals.
	*/
	Eigen::Matrix4f getICPRegistration(geodetect::Cloud& reference,
		geodetect::Cloud& source, float radius = 1.0);
}