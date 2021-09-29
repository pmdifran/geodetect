#pragma once
#include "ScalarField.h"
#include "log.h"

//PCL core
#include <pcl/point_types.h>

//KdTree (FLANN)
#include <pcl/search/kdtree.h>

//Octree
#include <pcl/octree/octree_search.h>
#include <pcl/search/octree.h>

//Filters
#include <pcl/filters/extract_indices.h>

#include <array>
#include <cmath>
#include <chrono>

// \brief  Point Cloud containing tools for Tools for processing and extracting information from natural environments. 
//  Built around PCL.
namespace geodetection
{
	class Cloud
	{
	//Members
	private:
		std::string m_name;

		//PCL XYZ Points and Normals
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
		pcl::PointCloud<pcl::Normal>::Ptr m_normals;

		//Scalar Fields (ScalarField is a wrapper around std::vector<float>)
		std::vector<ScalarField> m_scalarfields;

		//PCL Search Trees
		pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> m_octree;

		//Cloud properties
		Eigen::Matrix4d m_transformation = Eigen::Matrix4d::Identity(); //transformation matrix
		std::array<float, 3> m_view = { 0, 0, 0 }; //view for normal orientation

		//Scale and resoluton
		double m_resolution_avg = 0.0; //average resolution (i.e. point spacing) of the point cloud.
		double m_resolution_stdev = 0.0; //standard deviation of resolution (i.e. point spacing) of the cloud.
		double m_scale = 0.0;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//Constructors and assignments
	public:
		Cloud()
			: m_name("Cloud Default"),
			m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
			m_normals(new pcl::PointCloud<pcl::Normal>),
			m_kdtree(new pcl::search::KdTree<pcl::PointXYZ>),
			m_octree(1.0f) //resolution here doesn't matter. Import methods will call build anyways. 
		{
			GD_CORE_TRACE(":: Constructing empty GeoDetection Cloud...");
		}

		Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name = "Cloud Default")
			: m_name(name),
			m_cloud(cloud),
			m_normals(new pcl::PointCloud<pcl::Normal>),
			m_kdtree(new pcl::search::KdTree<pcl::PointXYZ>),
			m_octree(0.1f) ////resolution here doesn't matter. Constructor will alter it anyways.
		{
			GD_CORE_TITLE("GeoDetection Cloud Construction");
			GD_CORE_TRACE("Creating GeoDetection Cloud Object: '{0}' with {1} points", m_name, m_cloud->size());
			
			//Build the KdTree. GeoDetection Clouds use both KdTrees and Octrees. 
			buildKdTree();

			//Use KdTree to estimate the mean resolution of the cloud
			this->getResolution();
			buildOctree(this->getOptimalOctreeResolution());
			
			GD_CORE_INFO("--> GeoDetection Cloud Created\n");
		}

		Cloud(const Cloud&) = default;
		Cloud(Cloud&&) = default;

		Cloud& operator=(const Cloud&) = default;
		Cloud& operator=(Cloud&&) = default;

		~Cloud() = default;

	//Accessors
	public:
		inline std::string name() { return m_name; }

		//Point data
		inline pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud() const { return m_cloud; }
		inline pcl::PointCloud<pcl::Normal>::Ptr const normals() const { return m_normals; }
		inline std::vector <ScalarField>* const scalarfields() { return &m_scalarfields; }

		//Search trees
		inline pcl::search::KdTree<pcl::PointXYZ>::Ptr const kdtree() const { return m_kdtree; }
		inline const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree() const { return m_octree; }
		inline pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree() { return m_octree; } //need to make pcl's nearestKSearch method const.

		//Cloud properties
		inline Eigen::Matrix4d transformation() const { return m_transformation; }
		inline void printTransformation() const { std::cout << std::setprecision(16) << std::fixed << m_transformation << '\n' << std::endl; }
		inline std::array<float, 3> view() const { return m_view; }

		inline double resolution() const { return m_resolution_avg; }

	//Setters
	public:
		//Set scalar fields to another
		inline void setScalarFields(std::vector<ScalarField> sf) { std::copy(sf.begin(), sf.end(), m_scalarfields.begin()); }

		//Set cloud to another and rebuild KdTrees
		inline void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { m_cloud = std::move(cloud); buildKdTree(); m_normals->clear(); }

		//Set normals to another.
		inline void setNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals) { m_normals = std::move(normals); }

		//Set viewpoint for normal orientation.
		inline void setView(float x, float y, float z) { m_view[0] = x; m_view[1] = y; m_view[2] = z; }

		void setScale(float scale) { m_scale = scale; }

	//Checks
	public:
		inline bool hasCloud() const { return m_cloud->size() > 0; }
		inline bool hasNormals() const { return m_normals->size() > 0; }
		inline bool hasScalarFields() const { return m_scalarfields.size() > 0; }
		inline bool hasResolution() const { return m_resolution_avg > 0; }

	//METHODS
	public:
/************************************************************************************************************************************************//**
*  Search Tree Construction
****************************************************************************************************************************************************/
		/**
		* Constructs K-dimensional search trees for the point cloud.
		* @TODO: Testing ongoing. This method may soon be removed.
		*/
		void buildKdTree();

		inline float getOptimalOctreeResolution() const { return (sqrt(2.0) * m_resolution_avg) + m_resolution_stdev; }
		inline int getOptimalOctreeLeafPopulation(float radius) const 
		{
			return (M_PI * pow(m_resolution_avg, 2) * (radius, 2));
		}

		/**
		* Constructs octree search tree for the point cloud.
		* @param resolution: voxel size at greatest depth (i.e. smallest scale).
		*/
		void buildOctree(float resolution);

		/**
		* Constructs octree search tree for the point cloud.
		* @param resolution: voxel size at greatest depth (i.e. smallest scale).
		* @param max_leaf_population: maximum population of a voxel at the greatest depth. Used for dynamic octree structure. 
		*/
		void buildOctreeDynamic(float resolution, int max_leaf_population);

		/**
		* Constructs octree search tree for the point cloud. Uses resolution to determine appropriate leaf sizes \
		* (i.e. cubic voxel dimensions)
		*/
		inline void buildOctreeOptimalParams() { this->buildOctree(this->getOptimalOctreeResolution()); }

		/**
		* Constructs octree search tree for the point cloud. Uses resolution to determine appropriate leaf sizes \
		* (i.e. cubic voxel dimensions). Takes input of search radius size to determine maximum population of leaves for \
		* the dynamic structure.
		* @param radius: Search radius being used for the particular search application.
		*/
		inline void buildOctreeDynamicOptimalParams(float radius) 
		{ 
			this->buildOctreeDynamic(this->getOptimalOctreeResolution(), this->getOptimalOctreeLeafPopulation(radius));
		}

		/**
		* Constructs octree search tree for the point cloud. Uses resolution to determine appropriate leaf sizes \
		* (i.e. cubic voxel dimensions). Takes input k-nearest neighbors to determine maximum population of leaves for \
		* the dynamic octree structure.
		* @param k: number of nearest-neighbors for the search application.
		*/
		inline void buildOctreeDynamicOptimalParams(int k)
		{
			this->buildOctreeDynamic(this->getOptimalOctreeResolution(), k * sqrt(2.0f));
		}

/************************************************************************************************************************************************//**
*  Resolution, Downsampling, and Filtering
****************************************************************************************************************************************************/

		/**
		* Computes the local point cloud resolution (i.e. spacing), from a specified number of neighbors.
		* Internal: updates member m_resolution_avg (average cloud resolution).
		* @param k: the number of neighbors to use for determining local resolution (default=2).
		* @return Vector of local resolutions, consistent with point cloud indices.
		*/
		std::vector<float> getResolution(int num_neighbors = 1);

		/**
		* Generates a new, subsampled cloud, using a voxel filter that replaces voxel-level samples with their centroid.
		* Therefore, the output points will NOT be true measurements. (This is useful for downsampling in autoregistration).
		* The local cloud should be dense relative to the input voxel size.
		* @param voxel_size: voxel size used to filter. The filtering
		* \return Shared pointer to the subsampeld cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelDownSample(float voxel_size);

		/**
		* getVoxelDownsample, but directly modifies member m_cloud, resets KdTrees, and averages normals/scalar fields.
		* Internal: modifies m_cloud, KdTrees, Normals, Scalar Fields
		* @param distance: minimum distance between points
		*/
		void voxelDownSample(float voxel_size);

		/**
		* Generates a new, subsampled cloud, using a minimum distance (similar to CloudCompare).
		* @param distance: minimum distance between points
		* \return Shared pointer to the subsampled cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDistanceDownSample(float distance);

		/**
		* getDistanceDownsample, but directly modifies member m_cloud, resets KdTrees, and averages normals/scalar fields.
		* Internal: modifies m_cloud, KdTrees, Normals, Scalar Fields.
		* @param distance: minimum distance between points
		*/
		void distanceDownSample(float distance);

		/*
		* Filters NaN values from the point cloud.
		*/
		void removeNaN();

/***********************************************************************************************************************************************//**
*  Normal Estimation
***************************************************************************************************************************************************/
		/**
		* Compute normals with a radius search using octree. Uses OpenMP. Uses viewpoint <m_view> for orienting normals.
		* Neighborhoods are demeaned (i.e. moved to the origin) prior to covariance matrix and EVD calculations.
		* Safe to use for point clouds that are far from the origin.
		* @param radius: Radius for spherical neighbour search used for principle component analysis.
		* @return shared pointer to the computed normals.
		*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormalsRadiusSearch(float radius);

		/**
		* Compute normals from k-nearest neighbors, search using octree. Uses OpenMP. Uses viewpoint <m_view> for orienting normals.
		* Neighborhoods are demeaned (i.e. moved to the origin) prior to covariance matrix and EVD calculations.
		* Safe to use for point clouds that are far from the origin.
		* @param radius: Radius for spherical neighbour search used for principle component analysis.
		* @return shared pointer to the computed normals.
		*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormalsKSearch(int k);

		/**
		* calls getNormalsRadiusSearch and sets member normals to the result.
		* @param radius: Radius for spherical neighbour search used for principle component analysis.
		*/
		inline void updateNormalsRadiusSearch(float radius)
		{
			auto new_normals = this->getNormalsRadiusSearch(radius);
			this->setNormals(new_normals);
		}

		/**
		* calls getNormalsKSearch and sets member normals to the result.
		* @param radius: Radius for spherical neighbour search used for principle component analysis.
		*/
		inline void updateNormalsKSearch(int k)
		{
			auto new_normals = this->getNormalsKSearch(k);
			this->setNormals(new_normals);
		}

/***********************************************************************************************************************************************//**
*  Scalar Fields
***************************************************************************************************************************************************/
		/*
		* Appends another column of ScalarFields to member.
		*@param new_fields: ScalarField (needs to be the same length as m_cloud).
		*/
		inline void addScalarField(ScalarField&& new_fields)
		{
			if (new_fields.size() == m_cloud->size()) { m_scalarfields.push_back(std::move(new_fields)); }
			else { GD_CORE_ERROR(":: Scalar field size must agree with the cloud size"); }
		}

		/**
		* Removes column of scalar fields given an index.
		* @param index: Index of scalar field to remove (removes the largest index by default).
		*/
		inline void deleteScalarField(int index)
		{
			if (index < 0 || m_scalarfields.size() == 0 || index > (m_scalarfields.size() - 1))
			{
				GD_CORE_ERROR(":: Attempting to access a scalar field index that does not exist");  
				return;
			}

			m_scalarfields.erase(m_scalarfields.begin() + index);
		}

		inline void deleteFirstScalarField() { this->deleteScalarField(0); }
		inline void deleteLastScalarField() { this->deleteScalarField(m_scalarfields.size() - 1); }

/***********************************************************************************************************************************************//**
*  Spatial averaging: Normals and Scalar Fields
***************************************************************************************************************************************************/
		/**
		* Averages member normals (m_normals) within a defined search radius, at select subset of corepoints
		* @pararm radius: Spatial averaging radius.
		* @param corepoints: Subset corepoints at which to average scalar fields.
		*/
		void averageNormalsSubset(float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints);

		/**
		* Averages member normals (m_normals) within a defined search radius, at all points (m_cloud).
		* @pararm radius: Spatial averaging radius.
		*/
		void averageNormals(float radius);

		/**
		* Averages specific subset scalar fields within a defined search radius. Reduces size of fields to the subset size.
		* @param radius: Spatial averaging radius.
		* @param corepoints: Subset corepoints at which to average scalar fields.
		* @param field_index: i-th column of scalar fields.
		*/
		void averageScalarFieldSubset(float radius, int field_index, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints);

		/**
		* Averages specific scalar fields within a defined search radius, at all points. m_scalar_fields.size() remains constant.
		* @param radius: Spatial averaging radius.
		* @param field_index: i-th column of scalar fields.
		*/
		void averageScalarField(float radius, int field_index);

		/** 
		* Averages specific subset all scalar field columns, within a defined search radius. Reduces size of fields to the subset size.
		* @param radius: Spatial averaging radius.
		* @param corepoints: Subset corepoints at which to average scalar fields.
		*/
		void averageAllScalarFieldsSubset(float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints);

		/**
		* Averages all scalar fields within a defined search radius, at all points (m_cloud).
		* @param radius: Spatial averaging radius.
		*/
		void averageAllScalarFields(float radius);

/***********************************************************************************************************************************************//**
*  Transformations
***************************************************************************************************************************************************/
		/**
		* Updates the geodetection::Cloud <m_transformation> matrix, without translating the cloud.
		* Should be called if a function has transformed the cloud without updating the matrix (i.e. pcl::GeneralizedIterativeClosestPoint)
		* @param transformation: Eigen affine transformation matrix.
		*/
		void updateTransformation(const Eigen::Matrix4f& transformation);

		/**
		* Transforms the geodetection::Cloud and updates <m_transformation> matrix.
		* @param transformation: Eigen affine transformation matrix.
		*/
		void applyTransformation(const Eigen::Matrix4f& transformation);

/***********************************************************************************************************************************************//**
*  Keypoints and fast point feature histograms
***************************************************************************************************************************************************/
		/**
		* Computes intrinsic shape signature keypoints.
		* @return shared pointer to a pcl point cloud of ISS keypoints.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyPoints();

		/**
		* Computes fast point feature histograms.
		* @param keypoints: shared pointer to a pcl point cloud containing keypoints, at which fpf histograms are calculated for.
		* @return shared pointer to point cloud with fast point feature histograms.
		*/
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float radius);

		/**
		* Computes persistant fast point feature histograms which are unique at all scales.
		* @param[out] keypoints: Shared pointer to a pcl point cloud containing keypoints. *Mutates.* Non-unique keypoints are removed.
		* @param scales[in]: scales at which to compute the features, and test their uniqueness to the mean.
		* @return: std::pair. (First) Fast point feature histogram for the keypoints, which are unique at all scales.
		* (Second) Unique keypoints, corresonding to the fpfh's.
		*/
		std::pair<pcl::PointCloud<pcl::FPFHSignature33>, pcl::PointCloud<pcl::PointXYZ>::Ptr> getFPFHMultiscalePersistance
		(const pcl::PointCloud<pcl::PointXYZ>::Ptr const keypoints, std::vector<float>& scales);

/***********************************************************************************************************************************************//**
*  ASCII Output
***************************************************************************************************************************************************/
		/**
		* Writes the final transformation matrix to an ascii file. (Precision = 16)
		* @param filename: Output file path/name.
		*/
		void writeTransformation(std::string& filename);

		/**
		* Writes the geodetection::Cloud to an ASCII file (c-style)
		* @param filename: Output file path/name.
		* @param write_normals: Whether to write normals Nx, Ny, Nz (default=true)
		* @param write_scalarfields: Whether to write scalar fields (default=true)
		*/
		void writeAsASCII(const std::string& filename, bool write_normals = true, bool write_scalarfields = true);
	};

}