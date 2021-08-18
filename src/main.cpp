
//#define LOG_ALL_OFF
//GeoDetection includes
#include "PCLreadASCII.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
#include "log.h"

//stdlib includes
#include <iomanip>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/keypoints/iss_3d.h>


int main (int argc, char* argv[])
{
	GeoDetection::Log::Init();

	const char* source_file = "test_source.txt";
	const char* reference_file = "test_reference.txt";

	GD_TRACE("Source name: {0}", source_file);
	GD_TRACE("Reference name: {0}", reference_file);

	////Import point clouds into GeoDetection objects
	GeoDetection::Cloud source(PCLreadASCIIxyz(source_file), "Source");
	GeoDetection::Cloud reference(PCLreadASCIIxyz(reference_file), "Reference");

	//Create subsampled GeoDetection objects
	GeoDetection::Cloud source_down = GeoDetection::Cloud(source.getDistanceDownSample(0.25), "Downsampled Source");
	GeoDetection::Cloud reference_down = GeoDetection::Cloud(reference.getDistanceDownSample(0.25), "Downsampled Reference");

	//Use subsampled objects for Global Registration
	Eigen::Matrix4f global_transformation = globalRegistration(reference_down, source_down);

	std::cin.get();

	Eigen::Matrix4f icp_transformation = icpRegistration(reference_down, source_down);
	std::cout << std::setprecision(16) << std::fixed <<
		"ICP Transformation: \n\n" << icp_transformation << std::endl;

	std::cin.get();
}
