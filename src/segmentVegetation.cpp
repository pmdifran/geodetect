#include "segmentVegetation.h"
#include "features.h"
#include <omp.h>

namespace GeoDetection
{
	//Main functions
	void 
		segmentVegetation(GeoDetection::Cloud& geodetect)
	{
		int64_t cloud_size = geodetect.cloud()->size();

		//0-init because we sum the weighted scores
		ScalarField vegetation_scores(cloud_size, 0, "Vegetation_Scores"); 

		//Tree segmentation parameters
		std::vector<float> curve_scales =   { 0.50, 1.00, 1.50, 2.00, 2.50, 3.00 };
		std::vector<float> density_scales = { 1.25, 1.00, 0.75, 0.60, 0.45, 0.40 };
		std::vector<float> weights =       { 0.15, 0.15, 0.10, 0.10, 0.10, 0.30 };

		if (weights.size() != curve_scales.size() || weights.size() != density_scales.size())
		{
			GD_ERROR("Vegetation segmentation multi-scale parameters must be the same size");
			GD_WARN("Curvature scales: {0} | Density scales: {1} | Weights {2}", 
				curve_scales.size(), density_scales.size(), weights.size());
		}

		for (int i = 0; i < weights.size(); i++)
		{
			//Compute features at their respective scales.
			auto normals = geodetect.getNormalsRadiusSearch(curve_scales[i], false);
			GeoDetection::ScalarField curvatures = NormalsToCurvature(normals);
			GeoDetection::ScalarField densities = getVolumetricDensities(geodetect, density_scales[i]);

			//Calculate TREEZ index
			getVegetationScore(vegetation_scores, weights[i], curvatures, densities);
		}

		//Push GeoDetection::ScalarField to GeoDetection::Cloud::m_scalar_fields.
		geodetect.addScalarField(vegetation_scores);
	}

	void segmentVegetationSimplified(GeoDetection::Cloud& geodetect)
	{
		GD_TITLE("Vegetation Segmentation - simplified");
		int64_t cloud_size = geodetect.cloud()->size();

		//0-init because we sum the weighted scores
		ScalarField vegetation_scores(cloud_size, 0, "Vegetation_Scores");

		//Tree segmentation parameters
		std::vector<float> curve_scales = { 0.50, 1.00, 1.50, 2.00, 2.50, 3.00 };
		std::vector<float> density_scales = { 1.25, 1.00, 0.75, 0.60, 0.45, 0.40 };
		std::vector<float> weights = { 0.15, 0.15, 0.10, 0.10, 0.10, 0.30 };

		if (weights.size() != curve_scales.size() || weights.size() != density_scales.size())
		{
			GD_ERROR("Vegetation segmentation multi-scale parameters must be the same size");
			GD_WARN("Curvature scales: {0} | Density scales: {1} | Weights {2}",
				curve_scales.size(), density_scales.size(), weights.size());
		}

		//Calculate volumetric densities for all scales and store in a 2D vector.
		std::vector<GeoDetection::ScalarField> densities_multiscale = getVolumetricDensitiesMultiscale(geodetect, density_scales);

		//Determine curvature at a small enough scale 
		pcl::PointCloud<pcl::Normal>::Ptr normals = geodetect.getNormalsKSearch(7, false);
		GeoDetection::ScalarField curvatures = NormalsToCurvature(normals);
		normals = nullptr;

		//Calculate multiscale averaged curvatures
		std::vector<ScalarField> curvatures_multiscale = computeAverageFieldMultiscale(geodetect, curvatures, curve_scales);

		// Compute vegetation index using the input features
		for (int i = 0; i < weights.size(); i++)
		{
			getVegetationScore(vegetation_scores, weights[i], curvatures_multiscale[i], densities_multiscale[i]);
		}

		//Push GeoDetection::ScalarField to GeoDetection::Cloud::m_scalar_fields.
		geodetect.addScalarField(vegetation_scores);
	}

 }