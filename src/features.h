#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
//NORMAL CALCULATION

	void computeNormalDemeaned(const Cloud& geodetect, pcl::Normal& normal, const std::vector<int>& indices, const std::array<float, 3>& view);
	void computeDemeanedNormalRadiusSearch(const Cloud& geodetect, float radius, int point_index, pcl::Normal& normal, const std::array<float, 3>& view);
	void computeDemeanedNormalRadiusSearchOctree(const Cloud& geodetect, float radius, int point_index, pcl::Normal& normal, const std::array<float, 3>& view);
	void computeDemeanedNormalKSearch(const Cloud& geodetect, int k, int point_index, pcl::Normal& normal, const std::array<float, 3>& view);

//FEATURE-AVERAGING

	//Average-out normals around a given scale of core points. For entire cloud: set corepoints equal to cloud.
	pcl::PointCloud<pcl::Normal>::Ptr computeAverageNormals(const Cloud& geodetect,
		float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

	//Average-out scalar fields around a given scale of core points. For entire cloud: set corepoints equal to cloud.
	ScalarField computeAverageField(const Cloud& geodetect, const ScalarField& field, float scale,
		pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

//FEATURE CALCULATION - MULTISCALE
	//Get multiscale curvatures from various spherical scales
	std::vector<ScalarField> getCurvaturesMultiscale(const Cloud& geodetect, const std::vector<float>& scales);

	//Get multiscale densities from various spherical scales
	std::vector<ScalarField> getVolumetricDensitiesMultiscale(const Cloud& geodetect, const std::vector<float>& scales);

//FEATURE-AVERAGING - MULTISCALE

	//Average-out scalar fields around a given scale of core points, at numerous scales. 
	//For entire cloud: set corepoints equal to cloud.
	std::vector<ScalarField>
		computeAverageFieldMultiscale(const Cloud& geodetect, const ScalarField& field,
			const std::vector<float> scales, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

//FEATURES

	//Gets a vector of curvatures, taken from normals.
	ScalarField NormalsToCurvature(const pcl::PointCloud<pcl::Normal>::Ptr normals);

	//Gets volumetric densities at a specified scale (scale)
	ScalarField getVolumetricDensities(const Cloud& geodetect, float scale);

	void getVegetationScore(ScalarField& vegetation_scores, float weight, ScalarField& curvatures, ScalarField& densities);

}
