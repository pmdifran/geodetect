#include "features.h"

namespace GeoDetection
{
//HELPERS

	//Gets index mapping to the input vector if it was sorted by descending values
	std::vector<size_t> sortIndices(const std::vector<float>& vec)
	{
		std::vector<size_t> idx(vec.size());
		std::iota(idx.begin(), idx.end(), 0);
		std::stable_sort(idx.begin(), idx.end(), [&vec](size_t i1, size_t i2) {return vec[i1] > vec[i2]; });
		return idx;
	}

	float fieldSubsetAverage(const ScalarField& field, const std::vector<int>::iterator& iter_begin, const std::vector<int>::iterator& iter_end)
	{
		float average = 0;
		for (std::vector<int>::iterator iter = iter_begin; iter != iter_end; iter++)
		{
			average += field[*iter];
		}
		average /= iter_end - iter_begin;
		return average;
	}

//FEATURE-AVERAGING

	pcl::PointCloud<pcl::Normal>::Ptr
		computeAverageNormals(const Cloud& geodetect,
			float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing average normals with radius: {0} ...", scale);

		auto cloud = geodetect.cloud();
		auto tree = geodetect.flanntree();
		auto normals = geodetect.normals();

		//Check for correct inputs
		if (scale <= 0) { GD_CORE_ERROR(":: Invalid normal averaging radius inputted"); return nullptr; }
		if (tree->getInputCloud()->size() != cloud->size()) { GD_CORE_ERROR(":: KdTree pointer disagrees with the input cloud."); return nullptr; }
		if (corepoints != nullptr && corepoints->size() == 0) { GD_CORE_WARN(":: Input core points are empty."); return nullptr; }

		if (cloud->size() != normals->size())
		{
			GD_CORE_ERROR(":: Cannot average out normals. Input size does not agree with the cloud.");
		}

		//Average normals as new pointcloud
		pcl::PointCloud<pcl::Normal>::Ptr averaged_normals(new pcl::PointCloud<pcl::Normal>);
		averaged_normals->points.reserve(corepoints->size());

		//If no core points given, use full cloud
		if (corepoints == nullptr) { corepoints = cloud; }

#pragma omp parallel for
		for (int64_t i = 0; i < corepoints->size(); i++)
		{
			pcl::Normal n(0, 0, 0, 0);
			pcl::PointXYZ p(corepoints->points[i]);
			std::vector<int> ids;
			std::vector<float> sqdistances;
			tree->radiusSearch(p, scale, ids, sqdistances);

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


	ScalarField
		computeAverageField(const Cloud& geodetect, const ScalarField& field, float scale,
			pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing average field with scale: {0}", scale);

		auto cloud = geodetect.cloud();
		auto tree = geodetect.flanntree();

		if (scale <= 0) { GD_CORE_ERROR(":: Invalid scalar field averaging scale inputted"); }
		if (tree->getInputCloud()->size() != cloud->size()) { GD_CORE_ERROR(":: KdTree pointer disagrees with the input cloud."); }
		if (corepoints != nullptr && corepoints->size() == 0) { GD_CORE_WARN(":: Input core points are empty."); }
		if (cloud->size() != field.size()) { GD_CORE_ERROR(":: Cannot average out scalars. Input size does not agree with the cloud."); }

		//If no corepoints specified, assume to use the entire cloud.
		if (corepoints == nullptr) { corepoints = cloud; }

		ScalarField averaged_field(corepoints->size(), 0);

#pragma omp parallel for
		for (int64_t i = 0; i < corepoints->size(); i++)
		{
			float average = 0;
			pcl::PointXYZ p(corepoints->points[i]);
			std::vector<int> ids;
			std::vector<float> sqdistances;
			tree->radiusSearch(p, scale, ids, sqdistances);

			int num_ids = ids.size();

			for (int j = 0; j < num_ids; j++)
			{
				average += field[ids[j]];;
			}

			averaged_field[i] = average / num_ids;
		}

		return averaged_field;
	}

//FEATURE AVERAGING - MULTISCALE

		//Gets volumetric densities at numerous scales, using the largest scale query for the other scales.
	std::vector<ScalarField>
		getVolumetricDensitiesMultiscale(const Cloud& geodetect, const std::vector<float>& scales)
	{
		GD_CORE_TRACE(":: Computing volumetric densities from list of {0} scales...", scales.size());
		auto cloud = geodetect.cloud();
		auto flanntree = geodetect.flanntree();
		auto normals = geodetect.normals();
		std::vector<size_t> sort_map = sortIndices(scales); //Get index mapping to sorted version of radii

		//Get sphere volumes for each scale
		std::vector<float> volumes(scales.size());
		for (size_t i = 0; i < scales.size(); i++)
		{
			volumes[i] = (4.0f / 3.0f) * M_PI * pow(scales[i], 3);
		}

		//initialize 2d vector (scale, pointID)
		std::vector<ScalarField> all_densities(scales.size(), std::vector<float>(cloud->size()));

#pragma omp parallel for
		for (int64_t i = 0; i < cloud->size(); i++)
		{
			std::vector<float> sqdistances;
			std::vector<int> indices;

			//compute search for max scale
			int num_points = flanntree->radiusSearch(cloud->points[i], scales[sort_map[0]], indices, sqdistances);
			all_densities[sort_map[0]][i] = num_points / volumes[sort_map[0]];

			//Iterate through smaller neighborhoods and compute density as a subset of sqdistances
			for (int j = 1; j < scales.size(); j++)
			{
				float sq_scale = pow(scales[sort_map[j]], 2);
				auto iter_end = std::find_if(sqdistances.begin(), sqdistances.end(), [&sq_scale](float x) 
					{return x > sq_scale; });

				num_points = iter_end - sqdistances.begin();
				all_densities[sort_map[j]][i] = num_points / volumes[sort_map[j]];
			}
		}

		return all_densities;
	}

	std::vector<ScalarField>
		computeAverageFieldMultiscale(const Cloud& geodetect, const ScalarField& field,
			const std::vector<float> scales, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints /* = nullptr */)
	{
		GD_CORE_TRACE(":: Computing locally averaged fields from list of {0} scales...", scales.size());

		auto cloud = geodetect.cloud();
		auto flanntree = geodetect.flanntree();
		std::vector<size_t> sort_map = sortIndices(scales); //Get index mapping to sorted version of radii

		//initialize 2d vector (scale, pointID)
		std::vector<ScalarField> field_multiscale_averaged(scales.size(), std::vector<float>(cloud->size()));

#pragma omp parallel for
		for (int64_t i = 0; i < cloud->size(); i++)
		{
			//Compute spatial KdTree search at the largest scale
			std::vector<float> sqdistances;
			std::vector<int> indices;
			flanntree->radiusSearch(cloud->points[i], scales[sort_map[0]], indices, sqdistances);

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

//FEATURES - SINGLE SCALE

//Gets a vector of curvatures, taken from normals.
	GeoDetection::ScalarField
		NormalsToCurvature(const pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		GD_CORE_TRACE(":: Getting rate of change curvatures from normals...");
		GeoDetection::ScalarField curvatures(normals->size());

#pragma omp parallel for
		for (int64_t i = 0; i < normals->size(); i++)
		{
			curvatures[i] = normals->points[i].curvature;

			if (isnan(curvatures[i]))
			{
				GD_CORE_ERROR("NaN Detected at index: {0}", i);
			}
		}
		return curvatures;
	}

	//Gets volumetric densities at a specified scale (scale)
	GeoDetection::ScalarField
		getVolumetricDensities(const Cloud& geodetect, const float scale)
	{
		GD_CORE_TRACE(":: Getting volumetric densities at scale: {0} ...", scale);
		auto cloud = geodetect.cloud();
		auto flanntree = geodetect.flanntree();
		auto normals = geodetect.normals();

		double sphere_vol = (4.0f / 3.0f) * M_PI * pow(scale, 3);

		GeoDetection::ScalarField densities(cloud->size());

#pragma omp parallel for
		for (int64_t i = 0; i < cloud->size(); i++)
		{
			std::vector<float> distances;
			std::vector<int> indices;

			int num_points = flanntree->radiusSearch(cloud->points[i], scale, indices, distances);
			densities[i] = num_points / sphere_vol;
		}

		return densities;
	}


	void
		getVegetationScore(ScalarField& vegetation_scores, const float weight,
			ScalarField& curvatures, ScalarField& densities)

	{
		curvatures.normalizeMinMax();
		densities.normalizeMinMax();

#pragma omp parallel for
		for (int64_t i = 0; i < vegetation_scores.size(); i++)
		{
			float score = weight * (curvatures[i] - densities[i]);
			vegetation_scores[i] = vegetation_scores[i] + score; //scores can increase with each set of scales.
		}

	}

}
