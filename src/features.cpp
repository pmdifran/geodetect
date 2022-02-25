#include "features.h"
#include "core.h"

#include "progressbar.h"

#include <pcl/features/normal_3d.h>
#include <pcl/common/impl/transforms.hpp> //for transformation

#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/multiscale_feature_persistence.h>

namespace geodetect
{
/***********************************************************************************************************************************************//**
*  Helpers
***************************************************************************************************************************************************/
	//Create centered neighborhood, with input point becoming the new origin for neighborhood of indices.
	pcl::PointCloud<pcl::PointXYZ> getSubcloudAtOrigin(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& indices, const pcl::PointXYZ& point)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_centered;
		cloud_centered.reserve(indices.size());

		for (int i = 0; i < indices.size(); i++)
		{
			cloud_centered.push_back(cloud->points[indices[i]]);
		}

		Eigen::Affine3d transform = Eigen::Affine3d::Identity();
		transform.translation() << -point.x, -point.y, -point.z;
		pcl::transformPointCloud(cloud_centered, cloud_centered, transform);

		return cloud_centered;
	}

	//Get the average of fields, given iterators to indices (i.e. pointing to a particular neighborhood subset).
	float 
		fieldSubsetAverage(const ScalarField& field, const std::vector<int>::iterator& iter_begin, const std::vector<int>::iterator& iter_end)
	{
		float average = 0;
		for (std::vector<int>::iterator iter = iter_begin; iter != iter_end; iter++)
		{
			average += field[*iter];
		}
		average /= iter_end - iter_begin;
		return average;
	}

/***********************************************************************************************************************************************//**
*  Normal calculation
***************************************************************************************************************************************************/
	//Computes the normal vector for a neighborhood of points, given the input point, and view for orienting the normal vector.
	void
		computeNormal(const pcl::PointCloud<pcl::PointXYZ>& neighborhood, pcl::PointXYZ& point, pcl::Normal& normal, const std::array<float, 3>& view)
	{
		// Compute the point normal
		float curvature;
		Eigen::Vector4f n;
		pcl::computePointNormal(neighborhood, n, curvature);

		// Flip normals so they consistently point in one direction
		pcl::flipNormalTowardsViewpoint(point, view[0], view[1], view[2], n);

		// Set the normal to the result
		normal.normal_x = n[0];
		normal.normal_y = n[1];
		normal.normal_z = n[2];
		normal.curvature = curvature;
	}

	//Computes the normal vector for a subset of a neighborhood of points, given the input point, and view for orienting the normal vector.
	void
		computeNormal(const pcl::PointCloud<pcl::PointXYZ>& neighborhood, std::vector<int>& subindices,
			pcl::PointXYZ& point, pcl::Normal& normal, const std::array<float, 3>& view)
	{
		assert(!(neighborhood.size() < subindices.size())); //make sure subindices are lesser or equal to size.
		// Compute the point normal
		float curvature;
		Eigen::Vector4f n;
		pcl::computePointNormal(neighborhood, subindices, n, curvature);

		// Flip normals so they consistently point in one direction
		pcl::flipNormalTowardsViewpoint(point, view[0], view[1], view[2], n);

		// Set the normal to the result
		normal.normal_x = n[0];
		normal.normal_y = n[1];
		normal.normal_z = n[2];
		normal.curvature = curvature;
	}

	//Computes the normal vector for a neighborhood of points, by transforming the neighborhood to the origin prior to demeaning.
	//Prevents catostrophic cancellation that can occur during demeaning, when computing covariance matrix.
	//--> catostrophic cancellation can occur since eigen uses floats to compute the covariance matrix.
	void
		computeNormalAtOrigin(const Cloud& geodetect, pcl::Normal& normal, const std::vector<int>& indices, const std::array<float, 3>& view)
	{
		//get neighborhood with indices[0] being moved at the origin
		auto cloud = geodetect.cloud();
		pcl::PointXYZ& point = cloud->points[indices[0]];

		pcl::PointCloud<pcl::PointXYZ> subcloud = getSubcloudAtOrigin(geodetect.cloud(), indices, point);

		// Compute the point normal
		computeNormal(subcloud, point, normal, view);
	}

	//Computes the normal vector for a neighborhood of points, by transforming the neighborhood to the origin prior to demeaning.
	//Uses Octree for radius search.
	//Uses OpenMP.
	void
		computeNormalAtOriginRadiusSearch(const Cloud& geodetect, pcl::Normal& normal, float radius, int point_index, const std::array<float, 3>& view)
	{
		auto cloud = geodetect.cloud();
		pcl::PointXYZ& point = cloud->points[point_index];
		auto& octree = geodetect.octree();

		std::vector<int> indices;
		std::vector<float> sqdistances;

		//If not enough points found, set fields to NaN
		if (octree.radiusSearch(point, radius, indices, sqdistances) < 3)
		{
			std::fill_n(normal.data_n, 3, std::numeric_limits<float>::quiet_NaN());
			normal.curvature = std::numeric_limits<float>::quiet_NaN();
			return;
		}

		//Otherwise compute normal, first moving neighborhood to origin.
		computeNormalAtOrigin(geodetect, normal, indices, view);
	}

	//Computes the normal vector for a point using a radius search, with transformation of the neighborhood to the origin prior to demeaning.
	//Need to make pcl's octree::OctreePointCloudSearch::nearestKSearch method const.
	void 
		computeNormalAtOriginKSearch(Cloud& geodetect, pcl::Normal& normal, int k, int point_index, const std::array<float, 3>& view)
	{
		auto cloud = geodetect.cloud();
		pcl::PointXYZ& point = cloud->points[point_index];
		auto& octree = geodetect.octree();

		std::vector<int> indices(k);
		std::vector<float> sqdistances(k);

		//If not enough points found, set fields to NaN
		if (octree.nearestKSearch(point, k, indices, sqdistances) < 3)
		{
			std::fill_n(normal.data_n, 3, std::numeric_limits<float>::quiet_NaN());
			normal.curvature = std::numeric_limits<float>::quiet_NaN();
			return;
		}

		//Otherwise compute normal, first moving neighborhood to origin.
		computeNormalAtOrigin(geodetect, normal, indices, view);
	}

/***********************************************************************************************************************************************//**
*  Key points and Fast Point Feature Histograms
***************************************************************************************************************************************************/
//Calculates Intrinsic Shape Signature keypoints (uses OpenMP).
	pcl::PointCloud<pcl::PointXYZ>::Ptr getISSKeyPoints(float salient_radius, float non_max_radius, int min_neighbors,
		pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints, pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
		float max_eigenratio21 /* = 0.975 */, float max_eigenratio32 /* = 0.975 */)
	{
		GD_CORE_TRACE(":: Computing intrinsic shape signature keypoints...");
		Timer timer;

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

		iss_detector.setInputCloud(corepoints);
		iss_detector.setSearchSurface(search_surface);
		iss_detector.setSearchMethod(search_surface_tree);

		iss_detector.setSalientRadius(salient_radius); // neighborhood at which we determine the largest spatial variations
		iss_detector.setNonMaxRadius(non_max_radius); // radius for the application of the non maxima supression algorithm.
		iss_detector.setMinNeighbors(min_neighbors);
		
		// constraint of eigenvalue ratios, to exclude frames of ambiguous axex at locally symmetric points.
		iss_detector.setThreshold21(max_eigenratio21); // (eigen2 / eigen1) < threshold; and similar below.
		iss_detector.setThreshold32(max_eigenratio32);
		iss_detector.setNumberOfThreads(omp_get_num_procs());
		iss_detector.compute(*keypoints);

		GD_CORE_INFO("Number of Keypoints detected: {0}", keypoints->size());
		GD_CORE_WARN("--> Keypoint calculation time: {0} ms\n", timer.getDuration());

		return keypoints;
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float radius,
		pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
		pcl::PointCloud<pcl::Normal>::Ptr search_surface_normals)
	{
		GD_CORE_TRACE("Computing fast point feature histograms...");
		assert(search_surface->size() == search_surface_normals->size());
		Timer timer;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> computefpfh;

		computefpfh.setNumberOfThreads(omp_get_num_procs());
		computefpfh.setInputCloud(keypoints);
		computefpfh.setSearchSurface(search_surface);
		computefpfh.setInputNormals(search_surface_normals);
		computefpfh.setSearchMethod(search_surface_tree);
		computefpfh.setRadiusSearch(radius);

		computefpfh.compute(*fpfh);

		GD_CORE_WARN("--> FPFH calculation time: {0} ms\n", timer.getDuration());

		return fpfh;
	}

	std::pair<pcl::PointCloud<pcl::FPFHSignature33>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
		getFPFHMultiscalePersistance (const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, std::vector<float>& scales, float alpha,
			pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
				pcl::PointCloud<pcl::Normal>::Ptr search_surface_normals)
	{
		GD_CORE_TRACE("Computing multiscale persistant fast point feature histograms...");
		Timer timer;
		GD_CORE_TRACE(":: Keypoints: {0}", keypoints->size());

		//set up fpfh estimation object
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr
			fpfh_estimator(new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);;
		fpfh_estimator->setInputCloud(keypoints);
		fpfh_estimator->setSearchSurface(search_surface);
		fpfh_estimator->setSearchMethod(search_surface_tree);
		fpfh_estimator->setInputNormals(search_surface_normals);

		//set up feature persistance calculation object (uses the fpfh estimation object).
		pcl::MultiscaleFeaturePersistence <pcl::PointXYZ, pcl::FPFHSignature33> feature_persistance;
		feature_persistance.setScalesVector(scales);
		feature_persistance.setAlpha(alpha); //factor for selecting unique points outside of (mean) +/- Alpha * std-deviation. Parameter from examples.
		feature_persistance.setFeatureEstimator(fpfh_estimator);
		feature_persistance.setDistanceMetric(pcl::CS); //Norm used to compute distance of feature histograms from the mean histogram. 

		// Determine persistant (unique) features, (indices subset of keypoints).
		// A feature is considered persistent if it is 'unique' at all the scales
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr output_features(new pcl::PointCloud<pcl::FPFHSignature33>);
		auto unique_keypoint_indices = pcl::make_shared<pcl::Indices>();
		feature_persistance.determinePersistentFeatures(*output_features, unique_keypoint_indices);

		GD_CORE_INFO("Persistent features size: {0}", output_features->size());
		GD_CORE_INFO("DEBUG Indices size: {0}", unique_keypoint_indices->size());
		pcl::PointCloud <pcl::PointXYZ>::Ptr keypoints_unique(new pcl::PointCloud<pcl::PointXYZ>);

		//Remove non-unique keypoints and store into new cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
		extract_indices_filter.setInputCloud(keypoints);
		extract_indices_filter.setIndices(unique_keypoint_indices);
		extract_indices_filter.filter(*keypoints_unique);

		//Create output pair and return
		std::pair < pcl::PointCloud<pcl::FPFHSignature33>::Ptr, pcl::PointCloud <pcl::PointXYZ>::Ptr > output_pair;
		output_pair = std::make_pair(std::move(output_features), std::move(keypoints_unique));

		return output_pair;
	}

/***********************************************************************************************************************************************//**
*  Feature Averaging
***************************************************************************************************************************************************/
	//Average - out normals around a given scale of core points. Entire neighborhood used by default. 
	//Uses OpenMP.
	pcl::PointCloud<pcl::Normal>::Ptr
		computeAverageNormals(Cloud& geodetect,
			float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing average normals with radius: {0} ...", scale);

		auto cloud = geodetect.cloud();
		auto& octree = geodetect.octree();
		auto normals = geodetect.normals();

		//Check for correct inputs
		if (scale <= 0) { GD_CORE_ERROR(":: Invalid normal averaging radius inputted"); return nullptr; }
		if (corepoints != nullptr && corepoints->size() == 0) { GD_CORE_WARN(":: Input core points are empty."); return nullptr; }

		if (cloud->size() != normals->size())
		{
			GD_CORE_ERROR(":: Cannot average out normals. Input size does not agree with the cloud.");
		}

		//If no core points given, use full neighborhood
		if (corepoints == nullptr) { corepoints = cloud; }

		//Average normals as new pointcloud
		pcl::PointCloud<pcl::Normal>::Ptr averaged_normals(new pcl::PointCloud<pcl::Normal>);
		averaged_normals->points.reserve(corepoints->size());

#pragma omp parallel for
		for (int64_t i = 0; i < corepoints->size(); i++)
		{
			pcl::Normal n(0, 0, 0, 0);
			pcl::PointXYZ p(corepoints->points[i]);
			std::vector<int> ids;
			std::vector<float> sqdistances;
			octree.radiusSearch(p, scale, ids, sqdistances);

			int num_ids = ids.size();

			for (int j = 0; j < num_ids; j++)
			{
				n.normal_x += normals->points[ids[j]].normal_x;
				n.normal_y += normals->points[ids[j]].normal_y;
				n.normal_z += normals->points[ids[j]].normal_z;
				n.curvature += normals->points[ids[j]].curvature;
			}
			n.normal_x /= num_ids;
			n.normal_y /= num_ids;
			n.normal_z /= num_ids;
			n.curvature /= num_ids;

			averaged_normals->points[i] = n;
		}

		return averaged_normals;
	}

	//Average-out scalar field around a given scale of core points. Entire neighborhood used by default.
	//Uses OpenMP.
	ScalarField
		computeAverageField(Cloud& geodetect, const ScalarField& field, float scale,
			pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing average field with scale: {0}", scale);

		auto cloud = geodetect.cloud();
		auto& octree = geodetect.octree();

		if (scale <= 0) { GD_CORE_ERROR(":: Invalid scalar field averaging scale inputted"); }
		if (corepoints != nullptr && corepoints->size() == 0) { GD_CORE_WARN(":: Input core points are empty."); }
		if (cloud->size() != field.size()) { GD_CORE_ERROR(":: Cannot average out scalars. Input size does not agree with the cloud."); }

		//If no corepoints specified, assume to use the entire neighborhood.
		if (corepoints == nullptr) { corepoints = cloud; }

		ScalarField averaged_field(corepoints->size(), 0);

#pragma omp parallel for
		for (int64_t i = 0; i < corepoints->size(); i++)
		{
			float average = 0;
			pcl::PointXYZ p(corepoints->points[i]);
			std::vector<int> ids;
			std::vector<float> sqdistances;
			octree.radiusSearch(p, scale, ids, sqdistances);

			int num_ids = ids.size();

			for (int j = 0; j < num_ids; j++)
			{
				average += field[ids[j]];;
			}

			averaged_field[i] = average / num_ids;
		}

		return averaged_field;
	}

/***********************************************************************************************************************************************//**
*  Feature Calculaton
***************************************************************************************************************************************************/
//Returns a ScalarField of curvatures, taken from input normals.
	ScalarField
		NormalsToCurvature(const pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		GD_CORE_TRACE(":: Getting rate of change curvatures from normals...\n");
		ScalarField curvatures(normals->size());

#pragma omp parallel for
		for (int64_t i = 0; i < normals->size(); i++)
		{
			curvatures[i] = normals->points[i].curvature;
		}
		return curvatures;
	}

	//Returns a ScalarField of volumetric densities, determined at a particular scale (i.e. radius).
	ScalarField
		getVolumetricDensities(Cloud& geodetect, float scale)
	{
		GD_CORE_TRACE(":: Getting volumetric densities at scale: {0} ...\n", scale);

		auto cloud = geodetect.cloud();
		auto& octree = geodetect.octree();
		auto normals = geodetect.normals();

		double sphere_vol = getSphereVolume(scale);

		ScalarField densities(cloud->size());

#pragma omp parallel for
		for (int64_t i = 0; i < cloud->size(); i++)
		{
			std::vector<float> distances;
			std::vector<int> indices;

			int num_points = octree.radiusSearch(cloud->points[i], scale, indices, distances);
			densities[i] = num_points / sphere_vol;
		}

		return densities;
	}

	//Computes the vegetation index from the weight, curvatures, and volumetric densities.The vegetation indices are summed
	// --> hence, why you need to pass them into the function for computing (i.e. calculation happens inside the container).
	//Thus, different weights, curvatures, and volumetric densities should be provided for each scale.
	void
		getVegetationScore(ScalarField& vegetation_scores, float weight,
			ScalarField& curvatures, ScalarField& densities)

	{
		curvatures.NaNtoValue(0.0);
		curvatures.normalizeMinMax();
		densities.normalizeMinMax();

#pragma omp parallel for
		for (int64_t i = 0; i < vegetation_scores.size(); i++)
		{
			float score = weight * (curvatures[i] - densities[i]);
			vegetation_scores[i] += score; //scores can increase with each set of scales.
		}
	}


/***********************************************************************************************************************************************//**
*  Feature Calculaton - Multiscale
***************************************************************************************************************************************************/
	//Gets normals at numerous scales, reusing the largest scale query for computing the other scales.
	//Reuses the largest search neighborhood to reduce the search times. 
	//@TODO: Break some of the internals into helper functions to improve readability.
	//@TODO: Try storing 
	std::vector<ScalarField>
		getCurvaturesMultiscale(Cloud& geodetect, const std::vector<float>& scales)
	{
		GD_CORE_TRACE(":: Computing curvatures from list of {0} scales...", scales.size());
		Timer timer;

		//Get sqdistances of scales
		std::vector<double> sqscales = vectorGetSquared<float, double>(scales);

		//Sort the scales descending, and store the sorted index mapping
		std::vector<size_t> sort_map = sortIndicesDescending(scales); //Get index mapping to sorted version of radii
		int id_max = sort_map[0]; //mapping to the maximum search

		//Get geodetect::Cloud data members
		auto cloud = geodetect.cloud();
		auto& octree = geodetect.octree();
		auto normals = geodetect.normals();
		auto view = geodetect.view();
		int64_t size = cloud->size();

		//initialize 2d vector (scale, pointID)
		std::vector<ScalarField> all_curvatures(scales.size(), std::vector<float>(size));

		GD_PROGRESS(progress_bar, size);
#pragma omp parallel for schedule(dynamic,10)
		for (int64_t i = 0; i < size; i++)
		{
			pcl::Normal c_normal; 
			pcl::PointXYZ& c_point = cloud->points[i];

			//containers for largestest scale search
			std::vector<float> sqdistances;
			std::vector<int> indices;

			//compute search for max scale
			octree.radiusSearch(cloud->points[i], scales[id_max], indices, sqdistances);

			//Octree does not return sorted queries. We need them sorted (ascending).
			sortOctreeQuery(indices, sqdistances);

			//Get neighborhood that is moved to the origin for accurate normal estimation, sorted by proximity.
			pcl::PointCloud<pcl::PointXYZ> neighborhood = getSubcloudAtOrigin(cloud, indices, c_point);
			
			//Transfer indices for new neighboorhood subcloud. 
			std::iota(indices.begin(), indices.end(), 0);

			//Compute normal for the largest neighborhood.
			computeNormal(neighborhood, indices, c_point, c_normal, view);
			all_curvatures[id_max][i] = c_normal.curvature;

			//Iterate through smaller neighborhoods (descending) and compute normals as a subset of neighborhood
			for (int j = 1; j < scales.size(); j++)
			{
				int id = sort_map[j]; //ID for descending scale of the unsorted vector

				std::vector<int> subindices = getProximalIndices(indices, sqdistances, sqscales[id]);

				//compute normal for subset of neighborhood
				computeNormal(*cloud, subindices, c_point, c_normal, view);
				all_curvatures[id][i] = c_normal.curvature;
			}
			GD_PROGRESS_INCREMENT(progress_bar, cloud->size());
		}

		GD_CORE_WARN("--> Multiscale normale-rate-of-change curvature calculation time: {0} ms\n", timer.getDuration());
		return all_curvatures;
	}

	//Gets volumetric densities at numerous scales, using the largest scale query for the other scales.
	//Reuses the largest search neighborhood to reduce the search times. 
	std::vector<ScalarField>
		getVolumetricDensitiesMultiscale(Cloud & geodetect, const std::vector<float>&scales)
	{
		GD_CORE_TRACE(":: Computing volumetric densities from list of {0} scales...", scales.size());
		Timer timer;

		//Sort the scales descending, and store the sorted index mapping
		std::vector<size_t> sort_map = sortIndicesDescending(scales); //Get index mapping to sorted version of radii
		int id_max = sort_map[0]; //mapping to the maximum search

		//Get geodetect::Cloud data members
		auto cloud = geodetect.cloud();
		auto& octree = geodetect.octree();
		auto normals = geodetect.normals();
		int64_t size = cloud->size();

		//Get sqdistances of scales and scale volumes
		std::vector<double> sqscales = vectorGetSquared<float, double>(scales);
		std::vector<double> volumes = vectorGetSphereVolumes<float>(scales);

		//Initialize 2d vector (scale, pointID)
		std::vector<ScalarField> all_densities(scales.size(), std::vector<float>(size));

		GD_PROGRESS(progress_bar, size);
#pragma omp parallel for schedule(dynamic,10)
		for (int64_t i = 0; i < size; i++)
		{
			std::vector<float> sqdistances;
			std::vector<int> indices;

			//Compute search for max scale
			octree.radiusSearch(cloud->points[i], scales[id_max], indices, sqdistances);
			all_densities[id_max][i] = sqdistances.size() / volumes[id_max];

			//Octree does not return sorted queries. We need sqdistances to be sorted.
			std::sort(sqdistances.begin(), sqdistances.end());

			//Iterate through smaller neighborhoods and compute density as a subset of sqdistances
			for (int j = 1; j < scales.size(); j++)
			{
				int id = sort_map[j];
				float sq_scale = sqscales[id];

				//Find cutoff point (first element that is greater than the scale)
				auto iter_end = std::find_if(sqdistances.begin(), sqdistances.end(), [&sq_scale](float x)
					{return x > sq_scale; });

				//get density from number of points in the new scale neighborhood
				all_densities[id][i] = (iter_end - sqdistances.begin()) / volumes[id];
			}
			GD_PROGRESS_INCREMENT(progress_bar, cloud->size());
		}

		GD_CORE_WARN("--> Multiscale volumetric density calculation time: {0} ms\n", timer.getDuration());
		return all_densities;
	}

/***********************************************************************************************************************************************//**
*  Feature Averaging - Multiscale
***************************************************************************************************************************************************/
	//Averages - out scalar fields around a given scale of core points, at numerous scales.
	//Reuses the largest search neighborhood to reduce the search times. 
	std::vector<ScalarField>
		computeAverageFieldMultiscale(Cloud& geodetect, const ScalarField& field,
			const std::vector<float> scales, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing locally averaged fields from list of {0} scales...\n", scales.size());

		//Sort the scales descending, and store the sorted index mapping
		std::vector<size_t> sort_map = sortIndicesDescending(scales); //Get index mapping to sorted version of radii
		int id_max = sort_map[0]; //mapping to the maximum search

		//Get geodetect::Cloud data members
		auto cloud = geodetect.cloud();
		auto& octree = geodetect.octree();
		int64_t size = cloud->size();

		//Initialize 2d vector (scale, pointID)
		std::vector<ScalarField> field_multiscale_averaged(scales.size(), std::vector<float>(size));

		GD_PROGRESS(progress_bar, size);
#pragma omp parallel for schedule(dynamic,10)
		for (int64_t i = 0; i < size; i++)
		{
			//Compute spatial KdTree search at the largest scale
			std::vector<float> sqdistances;
			std::vector<int> indices;
			octree.radiusSearch(cloud->points[i], scales[sort_map[0]], indices, sqdistances);

			//compute average field at max scale (i.e. sort_map[0])
			field_multiscale_averaged[sort_map[0]][i] = fieldSubsetAverage(field, indices.begin(), indices.end());

			//Iterate through the smaller neighborhoods and compute average field as subset of the KdTree query.
			for (int j = 1; j < scales.size(); j++)
			{
				//Find iterator to the indices which include the current scale. 
				float sq_scale = pow(scales[sort_map[j]], 2);
				auto iter_end = std::find_if(sqdistances.begin(), sqdistances.end(), [&sq_scale](float x) 
					{return x > sq_scale; });
				std::vector<int>::iterator id_iter_end = indices.begin() + (iter_end - sqdistances.begin());

				field_multiscale_averaged[sort_map[j]][i] = fieldSubsetAverage(field, indices.begin(), id_iter_end);
			}
			GD_PROGRESS_INCREMENT(progress_bar, cloud->size());
		}

		return field_multiscale_averaged;
	}

}
