//geodetection
#include "GeoDetection.h"
#include "features.h" //for average scalar field compute
#include "progressbar.h"

#include <omp.h> //for Open MP

//stdlib
#include <chrono>
#include <fstream>
#include <iomanip> //for std::setprecision
#include <algorithm> //for std::remove

//for transformation
#include <pcl/common/impl/transforms.hpp>

//feature estimation
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/principal_curvatures.h>

//filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

//IO
#include <pcl/io/pcd_io.h>

namespace geodetection
{
/************************************************************************************************************************************************//**
*  Search Tree Construction
****************************************************************************************************************************************************/
	// Constructs K-dimensional search trees for the point cloud.
	void
		Cloud::buildKdTree()
	{
		GD_CORE_TRACE(":: Constructing K-dimensional Search Trees");
		Timer timer;

		m_kdtree->setInputCloud(m_cloud);
		m_kdtree->setSortedResults(true);

		GD_CORE_WARN("--> KdTree construction time: {0} ms\n", timer.getDuration());
	}

	//Constructs octree search trees for the point cloud.
	void
		Cloud::buildOctree(float resolution)
	{
		GD_CORE_TRACE(":: Constructing octrees");
		Timer timer;

		m_octree.deleteTree(); //delete previous tree (Cloud only stores one octree at a time)
		m_octree.setResolution(resolution);
		m_octree.setInputCloud(m_cloud);
		m_octree.addPointsFromInputCloud();

		GD_CORE_WARN("--> Octree construction time: {0} ms\n", timer.getDuration());
	}

	//Constructs octree search trees for the point cloud.
	//Uses dynamic depth, for maximum number of points in a leaf (dynamic depth), as well as a specified lead resolution.
	void
		Cloud::buildOctreeDynamic(float resolution, int max_leaf_population)
	{
		GD_CORE_TRACE(":: Constructing octrees");
		Timer timer;

		m_octree.deleteTree(); //delete previous tree (Cloud only stores one octree at a time)
		m_octree.setResolution(resolution);
		m_octree.enableDynamicDepth(max_leaf_population); //setting dynamic property of the octree. 
		m_octree.setInputCloud(m_cloud);
		m_octree.addPointsFromInputCloud();

		GD_CORE_WARN("--> Octree construction time: {0} ms\n", timer.getDuration());
	}

/************************************************************************************************************************************************//**
*  Resolution, Downsampling, and Filtering
****************************************************************************************************************************************************/
	//Computes the local point cloud resolution(i.e.spacing), from a specified number of neighbors, with OpenMP.
	//Returns a vector of the point resolutions (can be added to ScalarFields), and updates m_resolution_avg with the average.
	std::vector<float>
		Cloud::getResolution(const int num_neighbors)
	{
		GD_CORE_TRACE(":: Computing cloud resolution...");
		GD_CORE_TRACE("--> k-nearest neighbors used for distances: {0}", num_neighbors);
		Timer timer;

		std::vector<float> resolution(m_cloud->size());
		unsigned int k = num_neighbors + 1; //The first nearest neighbor is the query point itself --> use for searches.

		double avg_resolution = 0; //cloud-wide average resolution

		//Calculate resolution for each point, and determine average resolution
		GD_PROGRESS(progress_bar, m_cloud->size());
#pragma omp parallel for reduction(+: avg_resolution)
		for (int64_t i = 0; i < m_cloud->size(); i++)
		{
			double local_resolution = 0.0; //per-point localized resolution determined with number of neighbors (num_neighbors)
			std::vector<int> indices(k);
			std::vector<float> sq_distances(k);

			m_kdtree->nearestKSearch(i, k, indices, sq_distances);
			for (int j = 1; j < k; j++)
			{
				local_resolution += sq_distances[j];
			}

			local_resolution = sqrt(local_resolution) / (double)num_neighbors;
			resolution[i] = local_resolution;
			avg_resolution += local_resolution; //thread-safe with omp reduction

			GD_PROGRESS_INCREMENT(progress_bar);
		}

		avg_resolution = avg_resolution / (double)m_cloud->size();
		m_resolution_avg = avg_resolution;

		GD_INFO("--> Point cloud resolution: {0} meters", m_resolution_avg);
		GD_CORE_WARN("--> Resolution estimation time: {0} ms\n", timer.getDuration());

		//Calculate standard deviation of resolution.
		double deviation = 0.0;
#pragma omp parallel for reduction(+: deviation)
		for (int64_t i = 0; i < m_cloud->size(); i++)
		{
			deviation += pow(resolution[i] - avg_resolution, 2);
		}
		
		m_resolution_stdev = sqrt(deviation / m_cloud->size());
		return resolution;
	}

	//Generates a new, subsampled cloud, using a voxel filter that replaces voxel - level samples with their centroid.
	//Resulting points are not "true" or "original" measurements.
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		Cloud::getVoxelDownSample(float voxel_size)
	{
		GD_CORE_TRACE(":: Creating downsampled cloud with voxels...\n--> voxel filter size: {0}", voxel_size);
		Timer timer;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(m_cloud);
		grid.setLeafSize(voxel_size, voxel_size, voxel_size);
		grid.filter(*cloud_down);

		GD_CORE_WARN("-->   Full cloud size: {0}\n --> Downsampled cloud size: {1}",
			m_cloud->size(), cloud_down->size());

		GD_CORE_WARN("--> Downsample time: {0} ms\n", timer.getDuration());

		return cloud_down;
	}

	//Calls getVoxelDownSample but modifies members rather than returning a new point cloud.
	//Modifies cloud, trees, normals (averaged), and scalarfields (averaged).
	void
		Cloud::voxelDownSample(float voxel_size)
	{
		auto corepoints = getVoxelDownSample(voxel_size);
		double distance = (voxel_size / 2) * sqrt(3); //averaging radius relative to distance from centre to corner of cube

		//Average the normals and scalar fields using the new cloud as core points.
		if (this->hasNormals()) { this->averageNormalsSubset(distance, corepoints); }
		if (this->hasScalarFields()) { this->averageAllScalarFieldsSubset(distance, corepoints); }

		//set m_cloud to the corepoints and rebuild octree
		m_cloud = corepoints;
		this->buildKdTree();
		this->getResolution();
		this->buildOctreeOptimalParams();
	}

	//Returns a subsampled cloud, using a minimum distance. 
	//I chose to interate through points in order (less fast, but it is deterministic). This method cannot be both be paralelized and deterministic. 
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		Cloud::getDistanceDownSample(float distance)
	{
		GD_CORE_TRACE(":: Subsampling cloud by distance... \n--> Distance: {0}", distance);
		Timer timer;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		inliers->indices.reserve(m_cloud->size()); //conservatively reserve space for entire cloud indices. 

		std::vector<char> markers;
		markers.resize(m_cloud->size(), -1); //set to -1 by default **remember bool(-1) --> True**

		for (int64_t i = 0; i < m_cloud->size(); i++)
		{
			if (markers[i])
			{
				std::vector<int> IDRadiusSearch;
				std::vector<float> DistanceRadiusSearch;

				m_octree.radiusSearch(m_cloud->points[i], distance, IDRadiusSearch, DistanceRadiusSearch);

				if (IDRadiusSearch.size() > 1) {
					for (std::vector<int>::iterator it = IDRadiusSearch.begin() + 1; it != IDRadiusSearch.end(); ++it) {
						markers[*it] = false;
					}
				}

				inliers->indices.push_back(i);
			}
		}

		size_t size_original = m_cloud->size();
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(m_cloud);
		extract.setIndices(inliers);
		extract.filter(*cloud_down);

		GD_CORE_WARN("-->   Full cloud size: {0}\n --> Downsampled cloud size: {1}", m_cloud->size(), cloud_down->size());
		GD_CORE_WARN("--> Downsample time: {0} ms\n", timer.getDuration());

		return cloud_down;
	}

	//Calls getDistanceDownSample but modifies members rather than returning a new point cloud.
	//Modifies cloud, trees, normals (averaged), and scalarfields (averaged).
	void
		Cloud::distanceDownSample(float distance)
	{
		GD_CORE_TITLE("Subsampling GeoDetection Cloud");
		auto corepoints = this->getDistanceDownSample(distance);

		//Average the normals and scalar fields using the new cloud as core points.
		if (this->hasNormals()) { this->averageNormalsSubset(distance, corepoints); }
		if (this->hasScalarFields()) { this->averageAllScalarFieldsSubset(distance, corepoints); }

		//set m_cloud to the corepoints and rebuild octree
		m_cloud = corepoints;
		this->buildKdTree();
		this->getResolution();
		this->buildOctree(this->getOptimalOctreeResolution());
	}

	//Removes NaN from point cloud 
	//@TODO update other members (normals, scalarfields, trees) after cloud is filtered, to keep size the same across the board.
	void
		Cloud::removeNaN()
	{
		GD_CORE_TRACE(":: Removing NaN from Cloud.\n");

		m_cloud->is_dense = false;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, indices);
		m_cloud->is_dense = true;
	}

/***********************************************************************************************************************************************//**
*  Normal Estimation
***************************************************************************************************************************************************/
	//Computes normals with a radius search. 
	//Neighborhoods are transformed to the origin prior to demeaning during the covariance calculations, to prevent catostrophic cancellation
	pcl::PointCloud<pcl::Normal>::Ptr
		Cloud::getNormalsRadiusSearch(float radius)
	{
		GD_CORE_TRACE(":: Computing point cloud normals...\n:: Normal scale {0}", radius);
		GD_CORE_WARN(":: # Threads automatically set to the number of cores: {0}", omp_get_num_procs());
		Timer timer;

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		normals->resize(m_cloud->size());

		//Create optimal octree for search:
		this->buildOctreeDynamicOptimalParams(radius);

		// Iterate through each point and compute normals from demeaned neighborhoods.
		GD_PROGRESS(progress_bar, m_cloud->size());
#pragma omp parallel for
		for (int i = 0; i < m_cloud->size(); i++)
		{
			computeNormalAtOriginRadiusSearch(*this, normals->points[i], radius, i,  m_view);
			GD_PROGRESS_INCREMENT(progress_bar);
		}

		GD_CORE_WARN("--> Normal calculation time: {0} ms\n", timer.getDuration());

		return normals;
	}

	//Computes normals with a k-nearest neighbor search. 
	//Neighborhoods are transformed to the origin prior to demeaning during the covariance calculations, to prevent catostrophic cancellation
	pcl::PointCloud<pcl::Normal>::Ptr
		Cloud::getNormalsKSearch(int k)
	{
		GD_CORE_TRACE(":: Computing point cloud normals...\n:: Number of neighbors: {0}", k);
		GD_CORE_WARN(":: # Threads automatically set to the number of cores: {0}", omp_get_num_procs());
		Timer timer;

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		normals->resize(m_cloud->size());

		//Create optimal octree for search:
		this->buildOctreeDynamicOptimalParams(k);

		// Iterate through each point and compute normals from demeaned neighborhoods.
		GD_PROGRESS(progress_bar, m_cloud->size());
#pragma omp parallel for
		for (int i = 0; i < m_cloud->points.size(); i++)
		{
			computeNormalAtOriginKSearch(*this, normals->points[i], k, i, m_view);
			GD_PROGRESS_INCREMENT(progress_bar);
		}

		GD_CORE_WARN("--> Normal calculation time: {0} ms\n", timer.getDuration());

		return normals;
	}

/***********************************************************************************************************************************************//**
*  Spatial averaging: Normals and Scalar Fields
***************************************************************************************************************************************************/
	//Computes an average subset of normals within specified radius. Uses OpenMP.
	//Mostly used as helpers to reduce the size of normals when voxelDownSample(), or distanceDownSample() is called.
	void
		Cloud::averageNormalsSubset(float radius, pcl::PointCloud < pcl::PointXYZ>::Ptr corepoints)
	{
		m_normals = computeAverageNormals(*this, radius, corepoints);
	}

	//Computes a of normals within specified radius. Uses OpenMP.
	//Mostly used for simplified feature calculations (i.e. spatial averaging a single-scale feature, rather than multiscale feature calculation)
	void
		Cloud::averageNormals(float radius)
	{
		auto corepoints = m_cloud;
		m_normals = computeAverageNormals(*this, radius, corepoints);
	}
	
	//Computes an average subset of a singular scalar field. Uses OpenMP.
	//Mostly used as helpers to reduce the size of scalar fields when voxelDownSample(), or distanceDownSample() is called.
	void 
		Cloud::averageScalarFieldSubset(float radius, int field_index, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints)
	{
		//Check for correct inputs
		if (field_index > m_scalarfields.size() - 1 || field_index < 0)
		{
			GD_CORE_ERROR(":: Invalid scalar field index accessed."); return;
		}

		GD_CORE_TRACE(":: Averaging scalar field index: {0} with name: {1}", field_index, m_scalarfields[field_index].name);
		geodetection::ScalarField averaged_fields;
		averaged_fields = computeAverageField(*this, m_scalarfields[field_index], radius, corepoints);

		m_scalarfields[field_index] = std::move(averaged_fields);
	}

	//Computes an average subset of a singular scalar field. Uses OpenMP.
	void 
		Cloud::averageScalarField(float radius, int field_index)
	{
		this->averageScalarFieldSubset(radius, field_index, m_cloud);
	}

	//Computes an average subset of all scalar fields. Uses OpenMP.
	//Mostly used as helpers to reduce the size of scalar fields when voxelDownSample(), or distanceDownSample() is called.
	void 
		Cloud::averageAllScalarFieldsSubset(float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints)
	{
		GD_CORE_TRACE(":: Averaging all scalar fields...");
		for (int i = 0; i < m_scalarfields.size(); i++) { this->averageScalarFieldSubset(radius, i, corepoints); }
		GD_CORE_TRACE(":: All fields averaged");
	}

	//Computes an average of all scalar fields. Uses OpenMP.
	//Useful for simpified spatial averaging of multiple features. 
	void 
		Cloud::averageAllScalarFields(float radius)
	{
		this->averageAllScalarFieldsSubset(radius, m_cloud);
	}

/***********************************************************************************************************************************************//**
*  Registration
***************************************************************************************************************************************************/
	//Updates m_transformation, with matrix multiplication of an input transformation matrix.
	void
		Cloud::updateTransformation(const Eigen::Matrix4f& transformation)
	{
		GD_CORE_TRACE(":: Updating transformation...");
		Eigen::Matrix4d transformation_d = transformation.cast<double>(); //using doubles for more accurate arithmitic
		m_transformation = transformation_d * m_transformation; //eigen matrix multiplication
		m_transformation = transformation_d * m_transformation; //eigen matrix multiplication
	}

	//Transforms m_cloud and updates m_transformation
	void
		Cloud::applyTransformation(const Eigen::Matrix4f& transformation)
	{
		GD_CORE_TRACE(":: Applying transformation...");
		Timer timer;

		pcl::transformPointCloud(*m_cloud, *m_cloud, transformation);
		this->updateTransformation(transformation);

		GD_CORE_WARN("--> Transformation time: {0} ms\n", timer.getDuration());
	}

	//Calculates Intrinsic Shape Signature keypoints (uses OpenMP).
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		Cloud::getKeyPoints()
	{
		GD_CORE_TRACE(":: Computing intrinsic shape signature keypoints...");
		Timer timer;

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

		iss_detector.setSearchMethod(m_kdtree);

		//Revisit parameters. Automatic selection using scale, subsample scale?
		iss_detector.setSalientRadius(0.5);
		iss_detector.setNonMaxRadius(1.5);
		iss_detector.setMinNeighbors(5);

		iss_detector.setInputCloud(m_cloud);

		iss_detector.setThreshold21(0.975);
		iss_detector.setThreshold32(0.975);
		iss_detector.setNumberOfThreads(omp_get_num_procs());
		iss_detector.compute(*keypoints);

		GD_CORE_WARN("--> Keypoint calculation time: {0} ms\n", timer.getDuration());

		return keypoints;
	}

	//Compute fast point feature histograms. Uses OpenMP.
	//Must enable AVX in Properties --> C/C++ --> Enable Enhanced Instruction Set --> /arch:AVX. 
	//Otherwise you get heap corruption when it computes.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr
		Cloud::getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
	{
		GD_CORE_TRACE("Computing fast point feature histograms...");
		Timer timer;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> computefpfh;

		computefpfh.setNumberOfThreads(omp_get_num_procs());
		computefpfh.setInputCloud(keypoints);
		computefpfh.setInputNormals(m_normals);
		computefpfh.setSearchSurface(m_cloud);
		computefpfh.setSearchMethod(m_kdtree);
		computefpfh.setRadiusSearch(3.0);

		computefpfh.compute(*fpfh);

		GD_CORE_WARN("--> FPFH calculation time: {0} ms\n", timer.getDuration());

		return fpfh;
	}

/***********************************************************************************************************************************************//**
*  ASCII Output
***************************************************************************************************************************************************/

	//Write the 4x4 transformation matrix into an ascii file.
	void
		Cloud::writeTransformation(std::string& fname)
	{
		GD_CORE_TRACE(":: Writing RT file...\n");

		std::ofstream ofs;
		ofs.open(fname, std::ios::out | std::ios::binary | std::ios::trunc);
		ofs << std::fixed << std::setprecision(16) << m_transformation << std::endl;
		ofs.close();
	}

	//Output the geodetection Cloud into an ASCII file.
	//Option for whether to output normals and scalar fields.
	//@TODO: add option for outputing specific scalar fields.
	void Cloud::writeAsASCII(const std::string& filename_str, bool write_normals /* = true */, bool write_scalarfields /* = true */)
	{
		GD_TITLE("Exporting GeoDetection Cloud");
		GD_TRACE(":: File: {}", filename_str);
		Timer timer;

		//Using c-string for c-style IO
		const char* filename = filename_str.c_str();

		//Check for proper normals
		if (!this->hasNormals())
		{
			GD_CORE_ERROR(":: Normals of size {0} cannot be written alongside cloud of size {1}",
				m_normals->size(), m_cloud->size());
			write_normals = false;
		}

		//Check for proper scalar fields
		if (!this->hasScalarFields())
		{
			GD_CORE_ERROR(":: Cannot write scalar fields. No scalar fields have been added to the cloud.");
			write_scalarfields = false;
		}

		static const auto BUFFER_SIZE = 16 * 1024;
		char buf[BUFFER_SIZE + 1];

		std::fstream fstream;
		fstream.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);

		size_t i = 0;

		//Once this works, spin into helper functions.
		while (i < m_cloud->size()) {

			size_t increment = 0; //number of chars written by latest call of snprintf
			size_t total_increment = 0; //number of chars needed for the current i-th point.
			size_t cx = 0; //number of chars to be written (can be larger than BUFFER_SIZE)

			while (true)
			{
				//Print xyz to the buffer
				increment = snprintf(buf + cx, BUFFER_SIZE - cx, "%.8f %.8f %.8f", m_cloud->points[i].x,
					m_cloud->points[i].y, m_cloud->points[i].z);
				cx += increment;
				total_increment = increment; //total_increment is reset here.

				if (cx >= BUFFER_SIZE) { break; } //possible buffer overflow on next call of snprintf if this isn't checked.

				//Print normals to the buffer
				if (write_normals)
				{
					increment = snprintf(buf + cx, BUFFER_SIZE - cx, " %.8f %.8f %.8f", m_normals->points[i].normal_x,
						m_normals->points[i].normal_y, m_normals->points[i].normal_z);
					cx += increment;
					total_increment += increment;
					if (cx >= BUFFER_SIZE) { break; }
				}

				//Print scalar fields to the buffer
				if (write_scalarfields)
				{
					bool buffer_full = false;
					for (size_t sf = 0; sf < m_scalarfields.size(); sf++)
					{
						increment = snprintf(buf + cx, BUFFER_SIZE - cx, " %.8f", m_scalarfields[sf][i]);
						cx += increment;
						total_increment += increment;
						if (cx >= BUFFER_SIZE) { buffer_full = true;  break; }
					}
					if (buffer_full) { break; }
				}

				increment = snprintf(buf + cx, BUFFER_SIZE - cx, "\n");
				cx += increment;
				total_increment += increment;
				i++;

				if (cx >= BUFFER_SIZE) { i--; break; }

				if (i == m_cloud->size()) {
					total_increment = 0;
					break;
				}

			}

			fstream.write(buf, cx - total_increment);
		}
		fstream.close();

		GD_WARN("--> Data export time: {0} ms", timer.getDuration());
	}
}