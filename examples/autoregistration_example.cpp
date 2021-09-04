#include "readascii.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
#include "log.h"

//stdlib includes
#include <iomanip>

int main(int argc, char* argv[])
{
	GeoDetection::Log::Init();
	auto start = GeoDetection::Time::getStart();

	//these are hardcoded to the test files. You could also import based off of command line arguments.
	const char* source_file = "test_source.txt";
	const char* reference_file = "test_reference.txt";

	GD_TRACE("Source name: {0}", source_file);
	GD_TRACE("Reference name: {0}", reference_file);


	GeoDetection::AsciiReader reader;
	reader.setFilename(source_file);
	GeoDetection::Cloud source = reader.import();
	
	reader.setFilename(reference_file);
	GeoDetection::Cloud reference = reader.import();

	//Subsample the clouds
	source.distanceDownSample(0.15);
	reference.distanceDownSample(0.15);
	
	//Use subsampled objects for Global Registration
	Eigen::Matrix4f global_transformation = GeoDetection::getGlobalRegistration(reference, source);

	Eigen::Matrix4f icp_transformation = getICPRegistration(reference, source);

	GD_INFO("Final Matrix:");
	std::cout << source_down.transformation() << "\n" << std::endl;

	GD_WARN("Total time: {0} s \n", GeoDetection::Time::getDuration(start) / 1000);

}