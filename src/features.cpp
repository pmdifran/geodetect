#include "features.h"
#include "core.h"

#include <span>

#include <pcl/features/normal_3d.h>
#include <pcl/common/impl/transforms.hpp> //for transformation

namespace GeoDetection
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
		assert(!(neighborhood->size() < subindices.size())); //make sure subindices are lesser or equal to size.
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
		computeNormalAtOriginRadiusSearch(const Cloud& geodetect, float radius, int point_index, pcl::Normal& normal, const std::array<float, 3>& view)
	{
		auto cloud = geodetect.cloud();
		pcl::PointXYZ& point = cloud->points[point_index];
		auto octree = geodetect.octree();

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
	void
		computeNormalAtOriginKSearch(const Cloud& geodetect, int k, int point_index, pcl::Normal& normal, const std::array<float, 3>& view)
	{
		auto cloud = geodetect.cloud();
		pcl::PointXYZ& point = cloud->points[point_index];
		auto octree = geodetect.octree();

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
*  Feature Averaging
***************************************************************************************************************************************************/
	//Average - out normals around a given scale of core points. Entire neighborhood used by default. 
	//Uses OpenMP.
	pcl::PointCloud<pcl::Normal>::Ptr
		computeAverageNormals(Cloud& geodetect,
			float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing average normals with radius: {0} ...", scale);

		//build optimal Octree for this operation
		geodetect.buildOctreeDynamicOptimalParams(scale);

		auto cloud = geodetect.cloud();
		auto octree = geodetect.octree();
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

		//build optimal Octree for this operation
		geodetect.buildOctreeDynamicOptimalParams(scale);

		auto cloud = geodetect.cloud();
		auto octree = geodetect.octree();

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

		//build optimal Octree for this operation
		geodetect.buildOctreeDynamicOptimalParams(scale);

		auto cloud = geodetect.cloud();
		auto octree = geodetect.octree();
		auto normals = geodetect.normals();

		double sphere_vol = (4.0f / 3.0f) * M_PI * pow(scale, 3);

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
		auto start = Time::getStart();

		//Get sqdistances of scales
		std::vector<double> sqscales(scales.size());
		for (size_t i = 0; i < scales.size(); i++)
		{
			sqscales[i] = pow(scales[i], 2);
		}

		//Sort the scales descending, and store the sorted index mapping
		std::vector<size_t> sort_map = sortIndicesDescending(scales); //Get index mapping to sorted version of radii
		int id_max = sort_map[0]; //mapping to the maximum search

		//Build optimal Octree for the maximum scale
		geodetect.buildOctreeDynamicOptimalParams(scales[id_max]);

		//Get GeoDetection::Cloud data members
		auto cloud = geodetect.cloud();
		auto octree = geodetect.octree();
		auto normals = geodetect.normals();
		auto view = geodetect.view();

		//initialize 2d vector (scale, pointID)
		std::vector<ScalarField> all_curvatures(scales.size(), std::vector<float>(cloud->size()));

		GD_CORE_TRACE(":: Computing...");
#pragma omp parallel for collapse(2)
		for (int64_t i = 0; i < cloud->size(); i++)
		{
			pcl::Normal c_normal; //normal used to store multiscale calculations
			pcl::PointXYZ& c_point = cloud->points[i];

			//containers for largest search
			std::vector<float> sqdistances;
			std::vector<int> indices;

			//compute search for max scale
			int num_points = octree.radiusSearch(cloud->points[i], scales[id_max], indices, sqdistances);

			//Octree does not return sorted queries. We need them sorted (ascending).
			sortOctreeQuery(indices, sqdistances);

			//Get neighborhood that is moved to the origin for accurate normal estimation. (It is now ordered by proximity!)
			pcl::PointCloud<pcl::PointXYZ> neighborhood = getSubcloudAtOrigin(cloud, indices, c_point); //(indices are no longer needed)

			//Compute normal for the largest neighborhood.
			computeNormal(neighborhood, c_point, c_normal, view);
			all_curvatures[id_max][i] = c_normal.curvature;

			//Iterate through smaller neighborhoods (descending) and compute normals as a subset of neighborhood
			for (int j = 1; j < scales.size(); j++)
			{
				pcl::Normal cc_normal; //deeper local object for storing normal in parallel loop.

				int id = sort_map[j]; //ID for descending scale of the unsorted vector
				float sq_scale = sqscales[id];

				//use reverse iterators because we're descending. Find first element that is less than the scale.
				auto iter_end = std::find_if(sqdistances.begin(), sqdistances.end(), [&sq_scale](float x)
					{return x < sq_scale; });

				//pcl doesn't currently support passing iterators through to pcl::computePointNormal... so we have to pass a subset of indices
				size_t subcloud_end = iter_end - sqdistances.begin();
				std::vector<int> subindices(indices.begin(), indices.begin() + subcloud_end);

				//compute normal for subset of neighborhood :)
				computeNormal(neighborhood, c_point, cc_normal, view);
				all_curvatures[id][i] = c_normal.curvature;
			}
		}

		GD_CORE_WARN("--> Multiscale normale-rate-of-change curvature calculation time: {0} ms\n", GeoDetection::Time::getDuration(start));
		return all_curvatures;
	}

	//Gets volumetric densities at numerous scales, using the largest scale query for the other scales.
	//Reuses the largest search neighborhood to reduce the search times. 
	std::vector<ScalarField>
		getVolumetricDensitiesMultiscale(Cloud & geodetect, const std::vector<float>&scales)
	{
		GD_CORE_TRACE(":: Computing volumetric densities from list of {0} scales...", scales.size());
		auto start = Time::getStart();

		//Sort the scales descending, and store the sorted index mapping
		std::vector<size_t> sort_map = sortIndicesDescending(scales); //Get index mapping to sorted version of radii
		int id_max = sort_map[0]; //mapping to the maximum search

		//Build optimal Octree for the maximum scale
		geodetect.buildOctreeDynamicOptimalParams(scales[id_max]);

		//Get GeoDetection::Cloud data members
		auto cloud = geodetect.cloud();
		auto octree = geodetect.octree();
		auto normals = geodetect.normals();

		//Get sphere volumes for each scale
		std::vector<float> volumes(scales.size());
		for (size_t i = 0; i < scales.size(); i++)
		{
			volumes[i] = (4.0f / 3.0f) * M_PI * pow(scales[i], 3);
		}

		//Initialize 2d vector (scale, pointID)
		std::vector<ScalarField> all_densities(scales.size(), std::vector<float>(cloud->size()));

		GD_CORE_TRACE(":: Computing...");
#pragma omp parallel for collapse(2)
		for (int64_t i = 0; i < cloud->size(); i++)
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
				float sq_scale = pow(scales[id], 2);

				//Find first element that is less than the scale.
				auto iter_end = std::find_if(sqdistances.begin(), sqdistances.end(), [&sq_scale](float x)
					{return x < sq_scale; });

				//get density from number of points in the new scale neighborhood
				all_densities[id][i] = (iter_end - sqdistances.begin()) / volumes[id];
			}
		}

		GD_CORE_WARN("--> Multiscale volumetric density calculation time: {0} ms\n", GeoDetection::Time::getDuration(start));
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

		//Build optimal Octree for the maximum scale
		geodetect.buildOctreeDynamicOptimalParams(scales[id_max]);

		//Get GeoDetection::Cloud data members
		auto cloud = geodetect.cloud();
		auto octree = geodetect.octree();

		//Initialize 2d vector (scale, pointID)
		std::vector<ScalarField> field_multiscale_averaged(scales.size(), std::vector<float>(cloud->size()));

#pragma omp parallel for
		for (int64_t i = 0; i < cloud->size(); i++)
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
		}

		return field_multiscale_averaged;
	}

}
