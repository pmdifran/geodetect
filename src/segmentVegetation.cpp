#include "segmentVegetation.h"
#include "features.h"
#include <omp.h>

namespace geodetect
{
	//Segments vegetation using weighted differences of multiscale volumetric densities and curvatures.
	//Largest scale queries are reused to reduce the search times. 
	void 
		segmentVegetation(geodetect::Cloud& geodetect)
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

		//Calculate volumetric densities for all scales and store in a 2D vector.
		std::vector<geodetect::ScalarField> densities_multiscale = getVolumetricDensitiesMultiscale(geodetect, density_scales);
		std::vector<geodetect::ScalarField> curvatures_multiscale = getCurvaturesMultiscale(geodetect, curve_scales);

		for (int i = 0; i < weights.size(); i++)
		{
			//Calculate TREEZ index
			getVegetationScore(vegetation_scores, weights[i], curvatures_multiscale[i], densities_multiscale[i]);
		}

		//Push geodetect::ScalarField to geodetect::Cloud::m_scalarfields.
		geodetect.addScalarField(std::move(vegetation_scores));
	}

	//Segments vegetation using weighted differences of volumetric densities and curvatures, averaged at multiple scales.
	//Largest scale queries are reused to reduce the search times. 
	void segmentVegetationSimplified(geodetect::Cloud& geodetect)
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
		std::vector<geodetect::ScalarField> densities_multiscale = getVolumetricDensitiesMultiscale(geodetect, density_scales);

		//Determine curvature at a small enough scale 
		pcl::PointCloud<pcl::Normal>::Ptr normals = geodetect.getNormalsKSearch(12);
		geodetect::ScalarField curvatures = NormalsToCurvature(normals);
		normals = nullptr;

		//Calculate multiscale averaged curvatures
		std::vector<ScalarField> curvatures_multiscale = computeAverageFieldMultiscale(geodetect, curvatures, curve_scales);

		// Compute vegetation index using the input features
		for (int i = 0; i < weights.size(); i++)
		{
			getVegetationScore(vegetation_scores, weights[i], curvatures_multiscale[i], densities_multiscale[i]);
		}

		//Push geodetect::ScalarField to geodetect::Cloud::m_scalarfields.
		geodetect.addScalarField(std::move(vegetation_scores));
	}

 }