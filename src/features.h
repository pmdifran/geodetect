#pragma once
#include "GeoDetection.h"

namespace geodetection
{
/***********************************************************************************************************************************************//**
*  Normal calculation
***************************************************************************************************************************************************/
	/**
	* Computes the normal vector for a subcloud of points, given the input point, and view for orienting the normal vector.
	* @param[in] subcloud: The cloud used for normal estimation.
	* @param[in] point: the point at which we're calculating the normal. It should be included in subcloud as well. 
	* @param[out] normal: location where the normal is computed in place.
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeNormal(const pcl::PointCloud<pcl::PointXYZ>& subcloud, pcl::PointXYZ& point, pcl::Normal& normal, const std::array<float, 3>& view);

	/**
	* Computes the normal vector for a neighborhood of points, by transforming the neighborhood to the origin prior to demeaning.
	* @param[in] geodetect: geodetection::Cloud object.
	* @param[in] indices: Neighborhood indices, corresponding to geodetect.cloud().
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeNormalAtOrigin(const Cloud& geodetect, pcl::Normal& normal, const std::vector<int>& indices, const std::array<float, 3>& view);

	/**
	* Computes the normal vector for a point using a radius search, with transformation of the neighborhood to the origin prior to demeaning.
	* @param[in] geodetect: geodetection::Cloud object.
	* @param[in] radius: Scale of neighborhood search.
	* @param[in] point_index: Current point index, corresponding to the geodetection::Cloud.
	* @param[out] normal: Normal that is filled by the calculation.
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeNormalAtOriginRadiusSearch(const Cloud& geodetect, pcl::Normal& normal, float radius, int point_index, const std::array<float, 3>& view);

	/**
	* Computes the normal vector for a point using a radius search, with transformation of the neighborhood to the origin prior to demeaning.
	* @param[in] geodetect: geodetection::Cloud object.
	* @param[in] k: K-nearest neighbors used for neighborhood search.
	* @param[in] point_index: Current point index, corresponding to the geodetection::Cloud.
	* @param[out] normal: Normal that is filled by the calculation.
	* @param[in] view: viewpoint used to orient the resulting normal.
	*/
	void computeNormalAtOriginKSearch(Cloud& geodetect, pcl::Normal& normal, int k, int point_index, const std::array<float, 3>& view);

/***********************************************************************************************************************************************//**
*  Key points and Fast Point Feature Histograms
***************************************************************************************************************************************************/
	/**
	* Computes intrinsic shape signature keypoints.
	* @param[in] salient radius: Spherical neighborhood (i.e. scale) at which we determine the largest point variations within.
	* @paramin[in] non_max_radius: Radius for the application of the non maxima supression algorithm.
	* @param[in] min_neighbors: The minimum number of neighbors that has to be found while applying the non maxima suppression algorithm
	* @param[in] cloud: Point cloud (possible subsampled search_surface) used to compute the ISS keypoints.
	* @param[in] search_surface: Point cloud used to compute the ISS signatures at salient_radius scales.
	* @param[in] tree: Search tree corresponding to search_surface.
	* @return shared pointer to a pcl point cloud of ISS keypoints.
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr getISSKeyPoints(float salient_radius, float non_max_radius, int min_neighbors,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
		float max_eigenratio21 = 0.975, float max_eigenratio32 = 0.975);

	/**
	* Computes fast point feature histograms.
	* @param keypoints: Shared pointer to a pcl point cloud containing keypoints, at which fpf histograms are calculated for.
	* @param radius: Scale at to compute the histograms.
	* @param search_surface: point cloud to use when computing histograms.
	* @param search_surface_tree: kdtree for the search_surface.
	* @param search_surface_normals: normals of search_surface point cloud (sizes must correspond)
	* @return shared pointer to point cloud with fast point feature histograms.
	*/
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float radius,
		pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
		pcl::PointCloud<pcl::Normal>::Ptr search_surface_normals);

	/**
	* Computes persistant fast point feature histograms which are unique at all scales.
	* @param[out] keypoints: Shared pointer to a pcl point cloud containing keypoints. *Mutates.* Non-unique keypoints are removed.
	* @param[in] scales: scales at which to compute the features, and test their uniqueness to the mean.
	* @param[in] alpha: factor for selecting unique points outside of {mean +/- Alpha * std-deviation}. Exampes use 1.3.
	* @param[in] search_surface: Cloud used to compute histograms, using neighbors.
	* @param[in] search_surface_tree: Kdtree used to search the search surface.
	* @return: std::pair. (First) Fast point feature histogram for the keypoints, which are unique at all scales.
	* (Second) Unique keypoints, corresonding to the fpfh's.
	*/
	std::pair<pcl::PointCloud<pcl::FPFHSignature33>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> getFPFHMultiscalePersistance
	(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, std::vector<float>& scales, float alpha,
		pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
		pcl::PointCloud<pcl::Normal>::Ptr search_surface_normals);


/***********************************************************************************************************************************************//**
*  Feature Averaging
***************************************************************************************************************************************************/
	/**
	* Average-out normals around a given scale of core points. Entire cloud used by default.
	* @param geodetect: geodetection::Cloud object.
	* @param scale: Spatial averaging scale (radius).
	* @param corepoints: Subcloud used for computing the average normals. (Full resolution is used by default).
	* @return Shared pointer to averaged normals.
	*/
	pcl::PointCloud<pcl::Normal>::Ptr computeAverageNormals(Cloud& geodetect,
		float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

	/**
	* Average-out scalar field around a given scale of core points. Entire cloud used by default.
	* @param geodetect: geodetection::Cloud object.
	* @param scale: Spatial averaging scale (radius).
	* @param corepoints: Subcloud used for computing the average scalar fields. (Full resolution is used by default).
	* @return Shared pointer to averaged normals.
	*/
	ScalarField computeAverageField(Cloud& geodetect, const ScalarField& field, float scale,
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
	* @param geodetect: geodetection::Cloud object.
	* @param scale: nearest neighbor radius.
	* @return ScalarField of volumetric densities.
	*/
	ScalarField getVolumetricDensities(Cloud& geodetect, float scale);

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
	* @param geodetect: geodetection::Cloud object.
	* @param scales: Vector of scales (radii).
	* @return Vector of ScalarFields (i.e. curvatures).
	*/
	std::vector<ScalarField> getCurvaturesMultiscale(Cloud& geodetect, const std::vector<float>& scales);

	/**
	* Get multiscale volumetric densities given input of scales.
	* @param geodetect: geodetection::Cloud object.
	* @param scales: Vector of scales (radii).
	* @return Vector of ScalarFields (i.e. volumetric densities).
	*/
	std::vector<ScalarField> getVolumetricDensitiesMultiscale(Cloud& geodetect, const std::vector<float>& scales);

/***********************************************************************************************************************************************//**
*  Feature Averaging - Multiscale
***************************************************************************************************************************************************/
	
	/**
	* Averages-out scalar fields around a given scale of core points, at numerous scales.
	* @param geodetect: geodetection::Cloud object.
	* @param field: The particular geodetection::ScalarField being averaged.
	* @param scales: Vector of numerous scales for averaging.
	* @param corepoints: Subset of geodetection::Cloud at which the average fields are computed at. Full cloud is used by default.
	* @result Vector of ScalarFields (i.e. 2D vector), with each column being a particular averaged scale.
	*/
//
	//For entire cloud: set corepoints equal to cloud.
	std::vector<ScalarField>
		computeAverageFieldMultiscale(Cloud& geodetect, const ScalarField& field,
			const std::vector<float> scales, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

}
