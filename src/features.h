#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
/***********************************************************************************************************************************************//**
*  Normal calculation
***************************************************************************************************************************************************/
	/**
	* Computes the normal vector for a neighborhood of points, by transforming the neighborhood to the origin prior to demeaning.
	* @param[in] geodetect: GeoDetection::Cloud object.
	* @param[out] normal: Normal that is filled by the calculation.
	* @param[in] indices: Neighborhood indices, corresponding to geodetect.cloud().
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeNormalDemeaned(const Cloud& geodetect, pcl::Normal& normal, const std::vector<int>& indices, const std::array<float, 3>& view);

	/**
	* Computes the normal vector for a point using a radius search, with transformation of the neighborhood to the origin prior to demeaning.
	* @param[in] geodetect: GeoDetection::Cloud object.
	* @param[in] radius: Scale of neighborhood search.
	* @param[in] point_index: Current point index, corresponding to the GeoDetection::Cloud.
	* @param[out] normal: Normal that is filled by the calculation.
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeDemeanedNormalRadiusSearch(const Cloud& geodetect, float radius, int point_index, pcl::Normal& normal, const std::array<float, 3>& view);

	/**
	* Computes the normal vector for a point using a radius search, with transformation of the neighborhood to the origin prior to demeaning.
	* @param[in] geodetect: GeoDetection::Cloud object.
	* @param[in] k: K-nearest neighbors used for neighborhood search.
	* @param[in] point_index: Current point index, corresponding to the GeoDetection::Cloud.
	* @param[out] normal: Normal that is filled by the calculation.
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeDemeanedNormalKSearch(const Cloud& geodetect, int k, int point_index, pcl::Normal& normal, const std::array<float, 3>& view);

/***********************************************************************************************************************************************//**
*  Feature Averaging
***************************************************************************************************************************************************/
	/**
	* Average-out normals around a given scale of core points. Entire cloud used by default.
	* @param geodetect: GeoDetection::Cloud object.
	* @param scale: Spatial averaging scale (radius).
	* @param corepoints: Subcloud used for computing the average normals. (Full resolution is used by default).
	* @return Shared pointer to averaged normals.
	*/
	pcl::PointCloud<pcl::Normal>::Ptr computeAverageNormals(const Cloud& geodetect,
		float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

	/**
	* Average-out scalar field around a given scale of core points. Entire cloud used by default.
	* @param geodetect: GeoDetection::Cloud object.
	* @param scale: Spatial averaging scale (radius).
	* @param corepoints: Subcloud used for computing the average scalar fields. (Full resolution is used by default).
	* @return Shared pointer to averaged normals.
	*/
	ScalarField computeAverageField(const Cloud& geodetect, const ScalarField& field, float scale,
		pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

/***********************************************************************************************************************************************//**
*  Feature Calculaton
***************************************************************************************************************************************************/
	/**
	* Returns a ScalarField of curvatures, taken from input normals.
	* @param normals: pcl normals computed at an arbitrary scale (i.e. independent of this function), which contain curvatures.
	* @return ScalarField of curvatures.
	*/
	ScalarField NormalsToCurvature(const pcl::PointCloud<pcl::Normal>::Ptr normals);

	/**
	* Returns a ScalarField of volumetric densities, determined at a particular scale (i.e. radius).
	* @param geodetect: GeoDetection::Cloud object.
	* @param scale: nearest neighbor radius.
	* @return ScalarField of volumetric densities.
	*/
	ScalarField getVolumetricDensities(const Cloud& geodetect, float scale);

	/**
	* Computes the vegetation index from the weight, curvatures, and volumetric densities. The vegetation indices are summed.
	* Thus, different weights, curvatures, and volumetric densities should be provided for each scale.
	* @param[out] vegetation_scores: ScalarField where the vegetation indices are summed.
	* @param[in] weight: factor used to normalize the incremental vegetation index at the particular scale before summation.
	* @param[in] curvatures: normal-rate-of-change curvature, determined from getNormals and NormalsToCurvature.
	* @param[in] densities: volumetric densities, determined from getVolumetricDensities.
	*/
	void getVegetationScore(ScalarField& vegetation_scores, float weight, ScalarField& curvatures, ScalarField& densities);

/***********************************************************************************************************************************************//**
*  Feature Calculaton - Multiscale
***************************************************************************************************************************************************/
	/**
	* Get multiscale curvatures given input of scales.
	* @param geodetect: GeoDetection::Cloud object.
	* @param scales: Vector of scales (radii).
	* @return Vector of ScalarFields (i.e. curvatures).
	*/
	std::vector<ScalarField> getCurvaturesMultiscale(const Cloud& geodetect, const std::vector<float>& scales);

	/**
	* Get multiscale volumetric densities given input of scales.
	* @param geodetect: GeoDetection::Cloud object.
	* @param scales: Vector of scales (radii).
	* @return Vector of ScalarFields (i.e. volumetric densities).
	*/
	std::vector<ScalarField> getVolumetricDensitiesMultiscale(const Cloud& geodetect, const std::vector<float>& scales);

/***********************************************************************************************************************************************//**
*  Feature Averaging - Multiscale
***************************************************************************************************************************************************/
	
	/**
	* Averages-out scalar fields around a given scale of core points, at numerous scales.
	* @param geodetect: GeoDetection::Cloud object.
	* @param field: The particular GeoDetection::ScalarField being averaged.
	* @param scales: Vector of numerous scales for averaging.
	* @param corepoints: Subset of GeoDetection::Cloud at which the average fields are computed at. Full cloud is used by default.
	* @result Vector of ScalarFields (i.e. 2D vector), with each column being a particular averaged scale.
	*/
//
	//For entire cloud: set corepoints equal to cloud.
	std::vector<ScalarField>
		computeAverageFieldMultiscale(const Cloud& geodetect, const ScalarField& field,
			const std::vector<float> scales, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

}
