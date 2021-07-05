#include "PCLreadASCII.h"
#include "GeoDetection.h"
#include <iomanip>

int main (int argc, char* argv[])
{
	const char* fname = "test - Cloud.txt";
	std::cout << fname << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>());

	int count = PCLreadASCIIxyz(fname, cloudptr);
	std::cout << "count: " << count << std::endl;

	std::cout << "size: " << cloudptr->size() << std::endl;

	PCLwriteASCIIxyz("output.txt", cloudptr);
}