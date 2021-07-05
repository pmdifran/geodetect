#pragma once
#include <pcl/point_types.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <cmath>

// \brief Custom pointcloud object for geocomputation of LiDAR point clouds
class GeoDetection
{
public:
	std::string m_name = "Default";
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud = nullptr; //same as std::shared_ptr <pcl::PointCloud <pcl::PointXYZ> > = nullptr
	pcl::PointCloud<pcl::Normal>::Ptr m_normals = nullptr;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN = nullptr;
	Eigen::Matrix4d m_RT = Eigen::Matrix4d::Identity();

	float m_view[3] = { 0, 0, 0 };
	double m_resolution = 0.0; //average resolution of the point cloud.
	double m_scale = 0.0; //subsampled resolution of the point cloud by means of voxel filtering or minimuim distance. 


//Object creation
public:
	//Default constructor
	GeoDetection()
		:m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		m_normals(new pcl::PointCloud<pcl::Normal>),
		m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>)
	{};

	//Constructor copying cloud smart_ptr
	GeoDetection(std::string& name, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
		:m_name(name),
		m_cloud(cloud)
	{
		m_kdtreeFLANN->setInputCloud(m_cloud);
		pcl::PointCloud<pcl::Normal>::Ptr m_normals(new pcl::PointCloud<pcl::Normal>);
	}

	//Constructor for stealing the smart_ptr
	GeoDetection(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr&& cloud)
		:m_name(name),
		m_cloud(std::move(cloud))
	{
		cloud = nullptr;
	}

	//Copy constructor
	GeoDetection(GeoDetection& other)
		:m_name(other.m_name),
		m_cloud(other.m_cloud),
		m_normals(other.m_normals),
		m_kdtreeFLANN(other.m_kdtreeFLANN),
		m_RT(other.m_RT)
	{
		std::cout << "\nCloud: " << other.m_name << "copied to " << m_name << '\n' << std::endl;
	}

	GeoDetection(GeoDetection&& other) noexcept
		:m_name(std::move(other.m_name)),
		m_cloud(std::move(other.m_cloud)),
		m_normals(std::move(other.m_normals)),
		m_kdtreeFLANN(std::move(other.m_kdtreeFLANN))

	{
		other.~GeoDetection();
	}

	~GeoDetection() {}

//Initialization methods and checks
public:
	inline void setScale(float& scale) { m_scale = scale; }

	/** \brief Check that normals have been computed.
	*/
	inline bool hasNormals() { return  m_normals == nullptr; }

	inline void setView(float& x, float& y, float& z) { m_view[0] = x; m_view[1] = y; m_view[2] = z; }

//Methods
public: 

	void removeNaN();

	void writeRT(const char* fname);

	/** \brief Method for computing the local point cloud resolution (i.e. spacing).
	* \param[in] k: the number of neighbors to use for determining local resolution (default=3).
	* \param[out] vector containing local point resolution with matching indices. 
	*/
	std::vector<float> getResolution(const int& k = 3);

	/** \brief Method for subsampling the cloud object using a voxel filter. The local cloud should be dense relative to voxel size
	{i.e. specify it based on the point cloud resolution from getResolution()}.
* \param[in] voxelsize: cubic voxel size used to create average-point locations.
*/
	void VoxelDownSample(const float& voxelsize);

	/** \brief Method for generating a new, subsampled cloud, using a voxel filter. The local cloud should be dense relative to voxel size
	{i.e. specify it based on the point cloud resolution from getResolution()}.
* \param[in] voxelsize: cubic voxel size used to create average-point locations.
*/
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
	* //View point is set as 0,0,0 as default unless set with setView
* \param[in] nrad: Radius for spherical neighbour search used for principle component analysis.
* \param[in] normals (optional): object for which the results are written to. Otherwise, normals are written to m_normals member.
*/
	void computeNormals(const float& nrad);
	void computeNormals(const float& nrad, pcl::PointCloud<pcl::Normal>::Ptr& normals);

	void applyTransformation(Eigen::Matrix4f& transformation);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyPoints();

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints);

};

