#pragma once
#include <pcl/point_types.h>
#include <pcl\kdtree\kdtree_flann.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// \brief Custom pointcloud object for geocomputation of LiDAR point clouds
class PclProcessing
{
public:
	std::string m_name = "Default";
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud = nullptr; //same as std::shared_ptr <pcl::PointCloud <pcl::PointXYZ> > = nullptr
	pcl::PointCloud<pcl::Normal>::Ptr m_normals = nullptr;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN = nullptr;

//Object creation
public:
	//Default constructor
	PclProcessing()
		:m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		m_normals(new pcl::PointCloud<pcl::Normal>),
		m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>)
	{};

	//Constructor copying cloud smart_ptr
	PclProcessing(std::string& name, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
		:m_name(name),
		m_cloud(cloud)
	{
		m_kdtreeFLANN->setInputCloud(m_cloud);
		pcl::PointCloud<pcl::Normal>::Ptr m_normals(new pcl::PointCloud<pcl::Normal>);
	}

	//Constructor for stealing the smart_ptr
	PclProcessing(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr&& cloud)
		:m_name(name),
		m_cloud(std::move(cloud))
	{
		cloud = nullptr;
	}

	//Copy constructor
	PclProcessing(PclProcessing& other)
		:m_name(other.m_name),
		m_cloud(other.m_cloud),
		m_normals(other.m_normals),
		m_kdtreeFLANN(other.m_kdtreeFLANN)
	{
		std::cout << "\nCloud: " << other.m_name << "copied to " << m_name << '\n' << std::endl;
	}

	PclProcessing(PclProcessing&& other) noexcept
		:m_name(std::move(other.m_name)),
		m_cloud(std::move(other.m_cloud)),
		m_normals(std::move(other.m_normals)),
		m_kdtreeFLANN(std::move(other.m_kdtreeFLANN))

	{
		other.~PclProcessing();
	}

	~PclProcessing() {}

//methods
public:
	/** \brief Method for computing the local point cloud resolution (i.e. spacing).
	* \param[in] k: the number of neighbors to use for determining local resolution (default=3).
	* \param[out] vector containing local point resolution with matching indices. 
	*/
	std::vector<float> getResolution(const int& k = 3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelDownSample(const float& voxelsize);

	/** \brief Method for subsampling the cloud object using a minimum distance (similar to CloudCompare).
* \param[in] distance: minimum distance between points. 
*/
	void DistanceDownSample(const float& distance);


	/** \brief Method for generating a new, subsampled cloud, using a minimum distance (similar to CloudCompare).
* \param[in] distance: minimum distance between points
* \param[out] new, subsampled, pointcloud object (pointer).
*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr getDistanceDownSample(const float& distance);

	/** \brief Method for computing normals.
* \param[in] nrad: Radius for spherical neighbour search used for principle component analysis.
* \param[in] normals (optional): object for which the results are written to. Otherwise, normals are written to m_normals member.
*/
	void getNormals(const float& nrad);
	void getNormals(const float& nrad, pcl::PointCloud<pcl::Normal>::Ptr& normals);
	
	void getKeyPoints();

	void globalRegistration(PclProcessing reference);

};



