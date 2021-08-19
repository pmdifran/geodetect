#pragma once
#include "log.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <array>
#include <cmath>
#include <chrono>

// \brief Custom pointcloud object for geocomputation of LiDAR point clouds
namespace GeoDetection
{
	class Cloud
	{
		//Members
	public:
		std::string m_name;
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree;
		pcl::PointCloud<pcl::Normal>::Ptr m_normals;

		pcl::PointIndices::Ptr m_subindices;

		Eigen::Matrix4d m_transformation;

		std::array<float, 3> m_view = { 0, 0, 0 };
		double m_resolution = 0.0; //average resolution of the point cloud.
		double m_scale = 0.0; //subsampled resolution of the point cloud by means of voxel filtering or minimuim distance. 

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW //So that dynamic allocation returns aligned pointer.

	//Constructors and assignments
	public:
		Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name = "GeoDetection Default")
			: m_name(name),
			m_cloud(cloud),
			m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>),
			m_kdtree(new pcl::search::KdTree<pcl::PointXYZ>),
			m_normals(new pcl::PointCloud<pcl::Normal>),
			m_transformation(Eigen::Matrix4d::Identity())
			
		{
			GD_CORE_TITLE("GeoDetection Cloud Construction");
			GD_CORE_TRACE("Creating GeoDetection Cloud Object: '{0}' with {1} points", m_name, m_cloud->size());
			setKdTrees();
			GD_CORE_INFO("--> GeoDetection Cloud Created");
		}

		Cloud(const Cloud&) = default;
		Cloud(Cloud&&) = default;

		Cloud& operator=(const Cloud&) = default;
		Cloud& operator=(Cloud&&) = default;

		~Cloud() = default;

		//Setters and checkers
	public:
		//Sets the cloud to a new pcl::PointCloud, updates the KdTrees, and clears the normals. 
		inline void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { m_cloud = std::move(cloud); setKdTrees(); m_normals->clear(); }

		//I'm not sure if passing const lvalue reference is correct?
		inline void setNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals) { m_normals = std::move(normals); }

		void setKdTrees();

		void setScale(float scale) { m_scale = scale; }

		//Checks that normals have been computed.
		inline bool hasNormals() { return  m_normals->size() == m_cloud->size() && m_normals->size() != 0; }

		inline void setView(float x, float y, float z) { m_view[0] = x; m_view[1] = y; m_view[2] = z; }

		//Methods
	public:
		void removeNaN();

		void writeRT(const char* fname);

		/** \brief Method for computing the local point cloud resolution (i.e. spacing).
		* \param[in] k: the number of neighbors to use for determining local resolution (default=2).
		* \param[out] vector containing local point resolution with matching indices.
		* \param[out] updated m_resolution to contain the average resolution
		*/
		std::vector<float> getResolution(int num_neighbors = 2);

		/** \brief Method for generating a new, subsampled cloud, using a voxel filter. The local cloud should be dense relative to voxel size
		{i.e. specify it based on the point cloud resolution from getResolution()}.
	* \param[in] voxel_size: cubic voxel size used to create average-point locations.
	*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelDownSample(float voxel_size);

		/** \brief Method for generating a new, subsampled cloud, using a minimum distance (similar to CloudCompare).
	* \param[in] distance: minimum distance between points
	* \param[out] new, subsampled, pointcloud object (pointer).
	*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDistanceDownSample(float distance);

		/** \brief Method for computing normals.
		* //View point is set as 0,0,0 as default unless set with setView
	* \param[in] nrad: Radius for spherical neighbour search used for principle component analysis.
	* \param[in] normals (optional): object for which the results are written to. Otherwise, normals are written to m_normals member.
	*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormals(float nrad);

		void applyTransformation(const Eigen::Matrix4f& transformation);

		pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyPoints();

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints);

	};
}

