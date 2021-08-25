
//#define LOG_ALL_OFF
//GeoDetection includes
#include "PCLreadASCII.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
#include "segmentVegetation.h"
#include "log.h"

//stdlib includes
#include <iomanip>

int main (int argc, char* argv[])
{
	GeoDetection::Log::Init();
	auto start = GeoDetection::Time::getStart();

	const char* source_file = "test_source.txt";
	GeoDetection::Cloud source(GeoDetection::PCLreadASCIIxyz(source_file), "Source");
	//GeoDetection::Cloud source_down(source.getDistanceDownSample(0.15));

	GeoDetection::segmentVegetation(source);
	source.writeAsASCII("test_out.txt");

	//const char* source_file = "test_source.txt";
	//const char* reference_file = "test_reference.txt";
	//

	//GD_TRACE("Source name: {0}", source_file);
	//GD_TRACE("Reference name: {0}", reference_file);

	//////Import point clouds into GeoDetection objects
	//GeoDetection::Cloud source(GeoDetection::PCLreadASCIIxyz(source_file), "Source");
	//GeoDetection::Cloud reference(GeoDetection::PCLreadASCIIxyz(reference_file), "Reference");

	////Create subsampled GeoDetection objects
	//GeoDetection::Cloud source_down = GeoDetection::Cloud(source.getDistanceDownSample(0.25), "Downsampled Source");
	//GeoDetection::Cloud reference_down = GeoDetection::Cloud(reference.getDistanceDownSample(0.25), "Downsampled Reference");

	////Use subsampled objects for Global Registration
	//Eigen::Matrix4f global_transformation = GeoDetection::getGlobalRegistration(reference_down, source_down);

	//Eigen::Matrix4f icp_transformation = getICPRegistration(reference_down, source_down);

	//GD_INFO("Final Matrix:");
	//std::cout << source_down.transformation() << "\n" << std::endl;

	GD_WARN("Total time: {0} s \n", GeoDetection::Time::getDuration(start)/1000);


	std::cin.get();

}
