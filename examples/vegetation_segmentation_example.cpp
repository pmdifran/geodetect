
//GeoDetection includes
#include "PCLreadASCII.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
#include "segmentVegetation.h"
#include "log.h"

//stdlib includes
#include <iomanip>

int main(int argc, char* argv[])
{
	GeoDetection::Log::Init();
	auto start = GeoDetection::Time::getStart();

	const char* source_file = "treez_test.txt"; //hardcoded - if using this for other than testing, provide command line arguments.
	GeoDetection::Cloud source(GeoDetection::PCLreadASCIIxyz(source_file), "Source");

	source.distanceDownSample(0.1);

	GeoDetection::segmentVegetationSimplified(source);
	source.writeAsASCII("test_out.txt");


	GD_WARN("Total time: {0} s \n", GeoDetection::Time::getDuration(start) / 1000);

}

