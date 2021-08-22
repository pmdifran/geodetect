#pragma once
#include "log.h"

//PCL core
#include <pcl/point_types.h>

//KdTrees
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

//Filters
#include <pcl/filters/extract_indices.h>

#include <array>
#include <cmath>
#include <chrono>

// \brief Custom pointcloud object for geocomputation of LiDAR point clouds
namespace GeoDetection
{
	class Cloud
	{
		//Members
	private:
		//PCL Data Structures
		std::string m_name;
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree;
		pcl::PointCloud<pcl::Normal>::Ptr m_normals;

		//Scalar fields					         // If structured, first num_fields terms correspond to columns:
		bool m_fields_structured = false;        //{0,0}, {1,0}, {2,0}; {0,1}, {1,1}, {2,1}
		int m_num_fields = 0;		             
		std::vector<float> m_scalar_fields;		 //If not structured, data is entire columns followed by another:
												 //({0,0}, {0,1}, {0,2]..., {1,0}, {1,1}, {1,2}

		//Transformation Matrix
		Eigen::Matrix4d m_transformation;
		std::array<float, 3> m_view = { 0, 0, 0 };

		//Scale and resoluton
		double m_resolution = 0.0; //average resolution of the point cloud.
		double m_scale = 0.0;

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
			getKdTrees();
			GD_CORE_INFO("--> GeoDetection Cloud Created");
		}

		Cloud(const Cloud&) = default;
		Cloud(Cloud&&) = default;

		Cloud& operator=(const Cloud&) = default;
		Cloud& operator=(Cloud&&) = default;

		~Cloud() = default;

	//Accessors
	public:
		inline std::string name() { return m_name; }
		inline pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() { return m_cloud; }
		inline pcl::search::KdTree<pcl::PointXYZ>::Ptr tree() { return m_kdtree; }
		inline pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flanntree() { return m_kdtreeFLANN; }
		inline pcl::PointCloud<pcl::Normal>::Ptr normals() { return m_normals; }
		inline std::vector<float>& scalarfields() { return m_scalar_fields; }

	//Setters and checks
	public:
		//Sets the cloud to a new pcl::PointCloud, updates the KdTrees, and clears the normals. 
		inline void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { m_cloud = std::move(cloud); getKdTrees(); m_normals->clear(); }

		inline void setNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals) { m_normals = std::move(normals); }

		inline void setView(float x, float y, float z) { m_view[0] = x; m_view[1] = y; m_view[2] = z; }

		void setScale(float scale) { m_scale = scale; }

		inline bool hasNormals() { return  m_normals->size() == m_cloud->size() && m_normals->size() != 0; }

	//Methods
	public:
		/** \brief Method for adding another scalar field (i.e column) to the scalar fields.
		* \param[in] new_fields: float vector which should be the same length as # of points (this is checked for).
		* \return Internal: m_scalar_fields is modified to include an additional scalar field.
		*/
		void addScalarField(std::vector<float>& new_fields);

		/** \brief Method for structuring the scalar fields. Scalar fields should be structured to improve write methods.
		* \return Internal: m_scalar_fields is modified such that the scalar fields of a singular point are
		*					contiguous in memory (i.e. [1,1], [1,2], [1,3]; [2,1], [2,2], [2,3]
		*/
		void structureScalarFields();

		/** \brief Method for unstructuring the scalar fields. 
		* \return Internal: m_scalar_fields is modified such that all entries of one scalar field are followed by another.
		*/
		void unstructureScalarFields();

		/** \brief Method for setting up the Kd 3D search trees for the cloud. Two KdTrees are used:
		* a regular pcl implementation - used as the input search tree for various methods. 
		* a pcl wrapper for FLANN - used for searches, and not as input into various pcl compute methods. 
		* \return Internal: builds trees m_kdtree and m_kdtreeFLANN.
		*/
		void getKdTrees();

		/** \brief Method for computing normals. View point is set as 0,0,0 as default unless set with setView
		* \param[in] nrad: Radius for spherical neighbour search used for principle component analysis.
		* \param[in] set_m_normals: Whether to set member m_normals to the result (default = true).
		* \return shared pointer to the computed normals.
		*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormals(float nrad, bool set_m_normals = true);

		/** \brief Method for computing the local point cloud resolution (i.e. spacing).
		* \param[in] k: the number of neighbors to use for determining local resolution (default=2).
		* \return Vector of local resolutions, consistent with point cloud indices.
		Internal: updates member m_resolution (average cloud resolution).
		*/
		std::vector<float> getResolution(int num_neighbors = 2);

		/** \brief Method for generating a new, subsampled cloud, using a voxel filter. The local cloud should be dense relative to voxel size
		{i.e. specify it based on the point cloud resolution from getResolution()}.
		* \param[in] voxel_size: cubic voxel size used to create average-point locations.
		* \return Shared pointer to the subsampeld cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelDownSample(float voxel_size);

		/** \brief Method for generating a new, subsampled cloud, using a minimum distance (similar to CloudCompare).
		* \param[in] distance: minimum distance between points
		* \return Shared pointer to the subsampled cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDistanceDownSample(float distance);

		/** \brief Method for transforming the cloud.
		* \param[in] transformation: affine matrix.
		* \return internal: combines with m_transformation with matrix multiplication.
		*/

		pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyPoints();

		/** \brief Method for computing fast point feature histograms 
		* \param[in] shared pointer to a pcl point cloud containing keypoints
		*/
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints);

		/* \brief Method for filtering NaN values from the point cloud.
		*/

		void applyTransformation(const Eigen::Matrix4f& transformation);

		/** \brief Method for computing intrinsic shape signature keypoints.
		* \return shared pointer to a pcl point cloud.
		*/

		void removeNaN();

		/* \brief Method for writing the transformation matrix to an ascii file.
		* \param[in] fname: Output file path/name.
		*/
		
		void writeTransformation(const char* fname);
	};

}

