#include "PCLreadASCII.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
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
	//const char* source_file = "test_source.txt";
	//const char* reference_file = "test_reference.txt";

	//std::cout << "Source name: "  << source_file << std::endl;
	//std::cout << "Reference name: " << reference_file << std::endl;

	////Import point clouds into GeoDetection objects
	//GeoDetection source(PCLreadASCIIxyz(source_file), "Source");
	//GeoDetection reference(PCLreadASCIIxyz(reference_file), "Reference");

	////Create subsampled GeoDetection objects
	//GeoDetection source_down = GeoDetection(source.getDistanceDownSample(0.25), "Downsampled Source");
	//GeoDetection reference_down = GeoDetection(reference.getDistanceDownSample(0.25), "Downsampled Reference");

	////Use subsampled objects for Global Registration
	//Eigen::Matrix4f transformation = globalRegistration(reference_down, source_down);
	//std::cout << "Transformation: \n\n" << transformation << std::endl;

	//std::cout << "Done - waiting for user to close launch window" << std::endl;
	//std::cin.get();


	const char* source_file = "test_source.txt";
	GeoDetection source(PCLreadASCIIxyz(source_file), "Source");
	source.m_cloud = source.getDistanceDownSample(0.25);
	source.m_normals = source.getNormals(3.0);
	

/// Keypoints
	std::cout << "Computing intrinsic shape signature keypoints..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	iss_detector.setSearchMethod(source.m_kdtree);

	//Have option to use scale to automatically determine parameters.
	//Declare hardcoded parameters together as const so they are easy to find.
	iss_detector.setSalientRadius(0.5); //***************************************************************************
	iss_detector.setNonMaxRadius(1.5); //***************************************************************************
	iss_detector.setMinNeighbors(5);

	iss_detector.setInputCloud(source.m_cloud);

	iss_detector.setThreshold21(0.975); //from ryan kromer
	iss_detector.setThreshold32(0.975); //***************************************************************************
	iss_detector.setNumberOfThreads(omp_get_num_procs()); //Explicitly doing this because PCL has been throwing user error 1001.
	iss_detector.compute(*keypoints);

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Keypoints computed: " << keypoints->size() << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	
/// FPFH
	std::cout << "Computing fast point feature histograms..." << std::endl;
	start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> computefpfh;

	computefpfh.setNumberOfThreads(1);
	computefpfh.setInputCloud(keypoints);
	computefpfh.setInputNormals(source.m_normals);

	computefpfh.setSearchMethod(source.m_kdtree);
	computefpfh.setRadiusSearch(3.0); //function of voxel subsample? ***************************************************************************

	computefpfh.setSearchSurface(source.m_cloud);//subsample? ******************************************************************
	computefpfh.compute(*fpfh);

	timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Fast point feature histograms computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;

	std::cin.get();
}