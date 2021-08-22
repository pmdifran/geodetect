#include "segmentVegetation.h"
#include <omp.h>
#include <pcl/features/principal_curvatures.h>

namespace GeoDetection
{
	//Helpers
	std::vector<float> 
	NormalsToCurvature(pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		std::vector<float> curvatures(normals->size());

#pragma omp parallel for
		for (int64_t i = 0; i < normals->size(); i++)
		{
			curvatures[i] = normals->points[i].curvature;
		}
	}

	std::vector<float> 
	getVolumetricDensities(GeoDetection::Cloud geodetect, float rad)
	{
		auto cloud = geodetect.cloud();
		auto flanntree = geodetect.flanntree();
		auto normals = geodetect.normals();

		double sphere_vol = (4 / 3) / (M_PI * pow(rad, 3));

		std::vector<float> densities(cloud->size());

#pragma omp parallel for
		for (int64_t i = 0; i < cloud->size(); i++)
		{
			std::vector<float> distances;
			std::vector<int> indices;

			flanntree->radiusSearch(cloud->points[i], rad, indices, distances);
			densities[i] = (indices.size() - 1) / sphere_vol;
		}

		return densities;

	}

	void 
	normalizeMinMax(std::vector<float>& data)
	{
		const double min = *(std::min_element(data.begin(), data.end()));
		const double max = *(std::max_element(data.begin(), data.end()));

		const double range = max - min;

#pragma omp parallel for
		for (int64_t i = 0; i < data.size(); i++)
		{
			data[i] = (data[i] - min) / range;
		}
	}

	void
	getVegetationScore(std::vector<float> vegetation_scores, float weight,
						std::vector<float> curvatures, std::vector<float> densities)

	{
		normalizeMinMax(curvatures);
		normalizeMinMax(densities);

#pragma omp parallel for
		for (int64_t i = 0; i < vegetation_scores.size(); i++)
		{
			float score = weight * (curvatures[i] - densities[i]);
			vegetation_scores[i] = vegetation_scores[i] + score; //scores can increase with each set of scales.
		}

	}

	//Main functions
	void 
		segmentVegetation(GeoDetection::Cloud& geodetect, float rad)
	{
		int64_t cloud_size = geodetect.cloud()->size();
		std::vector<float> vegetation_scores(cloud_size, 0); //0-init because we sums the weighted scores

		//Tree segmentation parameters
		std::array<float, 6> curve_scale =   { 0.50, 1.00, 1.50, 2.00, 2.50, 3.00 };
		std::array<float, 6> density_scale = { 1.25, 1.00, 0.75, 0.60, 0.45, 0.40 };
		std::array<float, 6> weights =       { 0.15, 0.15, 0.10, 0.10, 0.10, 0.30 };

		if (weights.size() != curve_scale.size() || weights.size() != density_scale.size())
		{
			GD_ERROR("Vegetation segmentation multi-scale parameters must be the same size");
			GD_WARN("Curvature scales: {0} | Density scales: {1} | Weights {2}", 
				curve_scale.size(), density_scale.size(), weights.size());
		}
	
		for (int i = 0; i < weights.size(); i++)
		{
			//Get normal-rate-of change curvature
			auto normals = geodetect.getNormals(curve_scale[i], false);
			std::vector<float> curvatures = NormalsToCurvature(normals);
			
			//Get volumetric point density
			std::vector<float> densities = getVolumetricDensities(geodetect, density_scale[i]);

			//Calculate TREEZ index
			getVegetationScore(vegetation_scores, weights[i], curvatures, densities);

		}

		//add some sort of scalar field labelling system. 
		geodetect.addScalarField(vegetation_scores);

	}

	//IN DEVELOPMENT
//	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr getPrincipalCurvatures(GeoDetection::Cloud& geodetect, float rad)
//	{
//		//Setup curvature calculation object
//		pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> calc_curvatures;
//		calc_curvatures.setInputCloud(geodetect.cloud());
//		calc_curvatures.setInputNormals(geodetect.normals());
//		calc_curvatures.setSearchMethod(geodetect.tree());
//		calc_curvatures.setSearchSurface(geodetect.cloud());
//		calc_curvatures.setRadiusSearch(rad);
//
//		//Compute curvatures
//		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
//		calc_curvatures.compute(*curvatures);
//	}
//
//	std::vector<float> getNormalRateCurvature(GeoDetection::Cloud& geodetect, float rad)
//	{
//		auto principal_curvatures = getPrincipalCurvatures(geodetect, rad);
//		std::vector<float> normal_rate_curvature(principal_curvatures->size());
//
//		for (int64_t i = 0; i < principal_curvatures->size(); i++)
//		{
//			auto p = principal_curvatures->points[i];
//
//			//normal_rate_curvature
//		}
//
//
//	}
 }