#include "PCLreadASCII.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
#include <iomanip>

int main (int argc, char* argv[])
{
	const char* sname = "2020-11-23_S1-5.txt";
	//const char* rname = "2017-09-02_SEC2.txt";

	std::cout << "Source name: "  << sname << std::endl;
	//std::cout << "Reference name: " << rname << std::endl;


	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ref(new pcl::PointCloud<pcl::PointXYZ>());

	PCLreadASCIIxyz(sname, src);
	//PCLreadASCIIxyz(rname, ref);

	GeoDetection source(src);
	//GeoDetection reference(ref);

	std::vector<float> sres = source.getResolution(2);
	//std::vector<float> rres = reference.getResolution(2);

	source.DistanceDownSample(0.1);

 	source.computeNormals(1.5);

	//source.m_scale = source.m_resolution * 5;
	//reference.m_scale = reference.m_resolution * 5;

	//GeoDetection src_down = source.getDistanceDownSample(0.2);
	//PCLwriteASCIIxyz("DDownsample.txt", src_down.m_cloud);

	//float scale = source.m_resolution

	//source.computeNormals(1.0);
	//reference.computeNormals(1.0);

	//Eigen::Matrix4f transformation = globalRegistration(reference, source);
	//std::cout << "Transformation: \n\n" << transformation << std::endl;

	std::cout << "Done - waiting for user to close launch window" << std::endl;
	std::cin.get();
}