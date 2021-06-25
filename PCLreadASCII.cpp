// Functions for importing custom .txt pointcloud data, and converting it to a pcl type.

#include "PCLreadASCII.h" //includes our custom point cloud structures, and those from pcl.
#include <cstring> //for memchr
#include "fast_float.h"


#include <limits>
#include <cstdio>

uintmax_t getLineCount(char const* fname)
{
	FILE* file = fopen(fname, "r");

	if (file == NULL) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}
	
	static const auto BUFFER_SIZE = 16 * 1024;
	char buf[BUFFER_SIZE + 1];
	uintmax_t lines = 1;

	while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {

		if (bytes_read == std::numeric_limits<size_t>::max())
			std::cout << "read failed" << std::endl;
		if (!bytes_read)
			break;

		//memchr searches first x bytes of buffer and returns pointer to the first instance of '\n'. If none found, returns nullptr
		for (char* p = buf; (p = (char*)memchr(p, '\n', (buf + bytes_read) - p)); p++)				// --> nullptr is implictly converted into booleans false, which terminates the loop :)
			lines++;								  // ^^^^^^^^^^^^^^^^^ Gets the remaining # of addresses in our buffer
	}

	fclose(file);

	//If last character is newline, reduce the number of lines by 1
	std::ifstream in(fname);

	if (!in.is_open()) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	char ch;
	in.seekg(-1, std::ios_base::end);
	in.get(ch);

	if (ch == '\n') { lines--; }

	in.close();
	return lines;
}

bool
hasHeader(char const* fname)
{
	bool hasheader;

	//input filestream
	std::string fnamestr(fname);
	std::ifstream in(fnamestr);

	//objects for parsing
	pcl::PointXYZ xyz; // temporary point
	std::string line; //string object for getline

	in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
	if (!in.is_open()) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	//Check if the file has a header
	std::getline(in, line);
	std::istringstream iss(line); //construct into stringstream object
	if (iss >> xyz.x >> xyz.y >> xyz.z) { // stream extraction returns false if a non-numeric value is entered
		hasheader = false;
		std::cout << ":: No header detected." << std::endl;
	}
	else {
		hasheader = true;
		std::cout << ":: Header found." << std::endl;
	}

	in.close();
	return hasheader;
}

size_t
getNumColumns(const char* fname)
{
	size_t num_columns = 0;

	//input filestream
	std::string fnamestr(fname);
	std::ifstream in(fnamestr);

	//objects for parsing
	float temp = 0;
	std::string line; //string object for getline


	in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
	if (!in.is_open()) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	std::getline(in, line);
	std::getline(in, line); //take second line incase of header
	std::istringstream iss(line);
	
	while (iss >> temp) num_columns++;

	in.close();
	return num_columns;
}

char
getDelimeter(const char* fname)
{
	const char delimeters[] = { ',', ' ', ';', 0 };

	//input filestream
	std::string fnamestr(fname);
	std::ifstream in(fnamestr);

	//objects for parsing
	float temp = 0;
	std::string line; //string object for getline

	in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
	if (!in.is_open()) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	std::getline(in, line);
	std::getline(in, line); //take second line incase of header

	int i = 0;
	while (i < strlen(delimeters)){
		if (line.find(delimeters[i]) != std::string::npos)
			break;
		i++;
	}

	if (delimeters[i] == ' ') 
		std::cout << ":: Delimeter detected: " << "~space~" << std::endl;
	else 
		std::cout << ":: Delimeter detected: " << delimeters[i] << std::endl;

	in.close();
	return delimeters[i];
}


void 
PCLreadASCIIxyz(char const* fname, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudptr)
{
	//Determine number of points and reserve space for the pcl pointcloud object
	size_t num_points = getLineCount(fname);
	if (num_points < 2) {
		std::cout << "Needs more than 2 points to create a point cloud object.\n Exiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}
	if (hasHeader(fname)) { num_points--; }

	cloudptr->points.reserve(num_points);
	std::cout << ":: Number of points: " << num_points << '\n' << std::endl;

	size_t num_columns = getNumColumns(fname);
	if (num_columns < 3) {
		std::cout << ":: Number of columns detected: " << num_columns << "\nThe inputted file must contain XYZ coordinates."
			<< "Exiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	//Read a file stream and import into pcl::PointCloud
	std::cout << "Reading data..." << std::endl;
	auto start = getTime();

	FILE* file = fopen(fname, "r");

	if (file == NULL) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	static const auto BUFFER_SIZE = 16 * 1024;
	char buf[BUFFER_SIZE + 1];
	float value;

	while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {

		if (bytes_read == std::numeric_limits<size_t>::max())
			std::cout << "read failed" << std::endl;
		if (!bytes_read) //not sure if this is neccessary
			break;


		for (char* p = buf; (p = (char*)memchr(p, '\n', (buf + bytes_read) - p)); p++) {
																			//if bytes_read < buffer_size
			//get location of the next \n ... make sure this works for special case of no \n at the eof.
			//If the xyz point is cutoff by the end of the buffer, then reset the buffer location so we get it 100% with the next iteration.

			//Create three substrings separated by the delimiter.

			//Use fast_float to convert into floats.

			//Push into the pointcloud
		}
	}

	fclose(file);
}

	



	//	while (in >> xyz.x >> xyz.y >> xyz.z) {
	//		cloudptr->points.push_back(xyz);
	//	}
	//else {
	//	while (getline(in, line)) {
	//		std::istringstream iss(line);
	//		in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace

	//		iss >> xyz.x >> xyz.y >> xyz.z;
	//		cloudptr->points.push_back(xyz);
	//	}
	//}
	//auto stop = getTime();
	//auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
	//std::cout << "--> Import duration: " << duration.count() << " seconds" << std::endl;

	//in.close();

	//std::cout << ":: Data imported a PCL xyz pointcloud \n" << std::endl;
}




template <typename point_t>
void 
ASCIIreadXYZ(std::string& filename, std::vector<point_t>& xyzpoints)
{
	std::cout << "Reading XYZ Data..." << std::endl;

//objects for parsing
	point_t xyz; // temporary variable of custom type for parsing
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
	bool is_xyz;
	int num_columns = 0;
	float temp = 0;

	std::getline(in, line);
	std::getline(in, line); //second line to protect in case of header.
	std::istringstream c_iss(line);

	while (c_iss >> temp) num_columns++;

	if (num_columns < 3) {
		std::cout << ":: Number of columns detected: " << num_columns << "\nThe inputted file must contain XYZ coordinates."
			<< "Exiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	else if (num_columns == 3) {
		is_xyz = true;
		std::cout << ":: XYZ data detected." << std::endl;
	}
	else {
		is_xyz = false;
		std::cout << ":: Additional point fields detected: " << num_columns - 3 << "\n--> importing XYZ only." << std::endl;
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
	}

	else 
	{
		isheader = true;
		std::cout << ":: Header found." << std::endl;
	}

//Determine number of points and allocate memory
	size_t num_points = count - (int)isheader;
	std::cout << ":: Number of points: " << num_points << '\n' << std::endl;
	xyzpoints.reserve(num_points);
	
//Read the stream and import into fullpoints
	std::cout << "Reading data..." << std::endl;

	auto start = getTime();
	if (is_xyz) {
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
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
	std::cout << "--> Import duration: " << duration.count() << " seconds" << std::endl;

	in.close();
	
	std::cout << ":: File imported \n" << std::endl;
}

//Copy xyz coordinates from templated vector of structs to a PCL::Pointcloud<pcl::PointXYZ>::Ptr (smart, shared ptr)
void 
ASCIItoPCL(std::vector<XYZ>& xyzpoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr)
{
	//initialize point cloud shape. 
	cloudptr->width = xyzpoints.size();
	cloudptr->height = 1;	//'height' is 1 for unorganized (i.e not pixelated) point clouds.
	cloudptr->points.resize((cloudptr->width));

	for (std::size_t i = 0; i < cloudptr->width; i++)
	{
		cloudptr->points[i].x = xyzpoints[i].x;
		cloudptr->points[i].y = xyzpoints[i].y;
		cloudptr->points[i].z = xyzpoints[i].z;
	}
}

