// Functions for importing custom .txt pointcloud data, and converting it to a pcl type.

#include "importAscii.h" //includes our custom point cloud structures, and those from pcl.

//Generates XYZ data from a .txt pointcloud file.
void readXYZ(std::string& filename, std::vector<XYZ>& xyzpoints)
{
	std::cout << "Reading XYZ Data..." << std::endl;

//objects for parsing
	XYZ xyz; // temporary variable of custom type for parsing
	std::string line; //string object for getline and stringstreams 

	std::ifstream in(filename); //input file stream
	in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
	if (!in.is_open()) {
		std::cout << "Failed to find or open the file" << std::endl;
		std::cout << "Exiting program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

//get the number of lines in the file and store in count
	size_t count = 0;
	while (std::getline(in, line))
		count++;
	in.clear(); //clear the eof flag from the std::ifstream object
	in.seekg(0); //return to beginning of stream

//Determine how many columns are in the data.
	bool isxyz;
	int num_columns = 0;
	float temp = 0;

	std::getline(in, line);
	std::getline(in, line); //second line to protect in case of header.
	std::istringstream c_iss(line);

	while (c_iss >> temp) num_columns++;

	if (num_columns < 3) {
		std::cout << "The inputted file must contain XYZ coordinates." << std::endl;
		std::cout << "Exiting program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	else if (num_columns == 3) {
		isxyz = true;
		std::cout << ":: XYZ data detected." << std::endl;
	}
	else {
		isxyz = false;
		std::cout << ":: Additional point fields detected." << std::endl;
		std::cout << "   --> importing XYZ only." << std::endl;
	}
		
	in.clear();
	in.seekg(0);

//test to see if the file has a header
	bool isheader;
	std::getline(in, line); //first line
	std::istringstream iss(line); //construct into stringstream object

	if (iss >> xyz.x >> xyz.y >> xyz.z) { //if parsing works, there is not header.
		isheader = false;
		in.clear();
		in.seekg(0);
		std::cout << ":: No header detected." << std::endl;
		std::cout << ":: Continuing..." << std::endl;
	}

	else 
	{
		isheader = true;
		std::cout << ":: Header found." << std::endl;
		std::cout << ":: Continuing..." << std::endl;
	}

//Determine number of points and allocate memory
	size_t num_points = count - (int)isheader;
	std::cout << ":: Number of points: " << num_points << '\n' << std::endl;
	xyzpoints.reserve(num_points);
	
//Read the stream and import into fullpoints
	std::cout << "Reading data..." << std::endl;

	auto start = getTime();
	if (isxyz) {
		while (in >> xyz.x >> xyz.y >> xyz.z) {
			xyzpoints.push_back(xyz);
		}
	}
	else {
		while (getline(in, line)) {
			std::istringstream iss(line);
			iss >> xyz.x >> xyz.y >> xyz.z;
			xyzpoints.push_back(xyz);
		}
	}
	auto stop = getTime();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Import duration: " << duration.count() << " ms" << std::endl;

	in.close();
	
	std::cout << ":: File imported \n" << std::endl;
}

//Copy xyz coordinates from custom data struct 'fullPoint' to a PCL::Pointcloud<pcl::PointXYZ>::Ptr (smart, shared ptr)
void convert_toPCL(std::vector<XYZ>& xyzpoints, 
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr,unsigned int& num_points)
{
	//initialize point cloud shape. 
	cloudptr->width = num_points;
	cloudptr->height = 1;	//'height' is 1 for unorganized (i.e not pixelated) point clouds.
	cloudptr->points.resize((cloudptr->width));

	for (std::size_t i = 0; i < num_points; i++)
	{
		cloudptr->points[i].x = xyzpoints[i].x;
		cloudptr->points[i].y = xyzpoints[i].y;
		cloudptr->points[i].z = xyzpoints[i].z;
	}
}

