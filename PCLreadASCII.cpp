// Functions for importing custom .txt pointcloud data, and converting it to a pcl type.

#include "PCLreadASCII.h" //includes our custom point cloud structures, and those from pcl.
#include <cstring> //for memchr
#include <cstdlib> //for strtod
#include "fast_float.h" //Single header library version of fast_float

#include <limits>
#include <cstdio>

uintmax_t 
getLineCount(char const* fname)
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
	//input filestream
	std::ifstream in(fname);

	//objects for parsing
	pcl::PointXYZ xyz; // temporary point
	std::string line; //string object for getline

	if (!in.is_open()) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	//Check if the file has a header
	std::getline(in, line);
	std::istringstream iss(line); //construct into stringstream object
	iss.imbue(std::locale(std::locale(), new csv_reader())); //using locale that treats CSV delimiters as whitesapce.

	if (iss >> xyz.x >> xyz.y >> xyz.z) { // stream extraction returns false if a non-numeric value is entered
		std::cout << ":: No header detected." << std::endl;
		return false;
	}
	else {
		std::cout << ":: Header found." << std::endl;
		return true;
	}
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

	if (!in.is_open()) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	std::getline(in, line);
	std::getline(in, line); //take second line incase of header
	std::istringstream iss(line);
	iss.imbue(std::locale(std::locale(), new csv_reader())); //treat CSV delimiters as whitespace
	
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


pcl::PointCloud<pcl::PointXYZ>::Ptr
PCLreadASCIIxyz(char const* fname)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>);

	auto start = std::chrono::steady_clock::now();
	std::cout << "\n\nImporting XYZ Data.." << std::endl;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Check that there is XYZ data
	size_t num_columns = getNumColumns(fname);
	if (num_columns < 3) {
		std::cout << ":: Number of columns detected: " << num_columns << "\nThe inputted file must contain XYZ coordinates."
			<< "Exiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Determine number of points and reserve space for the pcl pointcloud object
	size_t num_points = getLineCount(fname);
	if (num_points < 2) {
		std::cout << "Needs more than 2 points to create a point cloud object.\n Exiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}
	bool hasheader = hasHeader(fname);
	if (hasheader) { num_points--; }

	cloudptr->points.reserve(num_points);
	std::cout << ":: Number of points: " << num_points << std::endl;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Read file into buffer (c-style because its faster) and import into pcl::PointCloud
	std::cout << ":: Reading data..." << std::endl;

	FILE* file = fopen(fname, "rb");

	if (file == NULL) {
		std::cout << "Failed to find or open the file\nExiting the program..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	static const auto BUFFER_SIZE = 16 * 1024;
	char buf[BUFFER_SIZE + 1];
	pcl::PointXYZ mypoint;
	
	//If there's a header, skip the first line.
	if (hasheader) {
		char* str = fgets(buf, BUFFER_SIZE, file);
	}

	while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {
		
		char* p = buf; //pointer to the beginning of the line that we'll read

		if (bytes_read == std::numeric_limits<size_t>::max())
			std::cout << "read failed" << std::endl;
		if (!bytes_read) //not sure if this is neccessary
			break;

		for (char* p_next = buf; (p_next = (char*)memchr(p_next, '\n', (buf + bytes_read) - p_next)); p_next++) {
			auto answer = fast_float::from_chars(p, p_next, mypoint.x);
			answer = fast_float::from_chars(answer.ptr + 1, p_next, mypoint.y);
			fast_float::from_chars(answer.ptr + 1, p_next, mypoint.z);

			cloudptr->points.push_back(mypoint);

			p = p_next + 1;
		}
			// CASE: The xyz point is split across the buffer. Reset the file location to include it at the beginning of the next buffer fill.
		if (bytes_read == BUFFER_SIZE) {
			size_t offset = (buf + bytes_read) - p; //number of bytes from the end of buffer to the preceding char to the final \n. 
			fseek(file, (long)-offset, SEEK_CUR); //offset the stream back that many bytes, so we're at the final \n in the next memchr.
			continue;
		}
			// EOF CASE: The last line in the file didn't terminate with \n
		else if (cloudptr->points.size() < cloudptr->points.capacity()) { 
			char* p_end; //pointer which lands on the delimeters.

			mypoint.x = strtod(p, &p_end);
			mypoint.y = strtod(p_end + 1, &p_end);
			mypoint.z = strtod(p_end + 1, NULL);
			cloudptr->points.push_back(mypoint);

		}
	}

	fclose(file);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "--> Data import time: "<< timer.count() << " seconds\n" << std::endl;
	return cloudptr;
}

void
PCLwriteASCIIxyz(char const* fname, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudptr)
{
	auto start = std::chrono::steady_clock::now();
	std::cout << "Exporting XYZ data..." << std::endl;

	static const auto BUFFER_SIZE = 16 * 1024;
	char buf[BUFFER_SIZE + 1];

	std::fstream fstream;
	fstream.open(fname, std::ios::out | std::ios::binary | std::ios::trunc);

	size_t i = 0;

	while (i < cloudptr->size()) {
		size_t cx = 0;
		size_t cx_increment = 0;

		while (true)
		{
			cx_increment = snprintf(buf + cx, BUFFER_SIZE - cx, "%.8f %.8f %.8f\n", cloudptr->points[i].x,
				cloudptr->points[i].y, cloudptr->points[i].z);
			
			cx += cx_increment;
			i++;

			if (cx >= BUFFER_SIZE) {
				i--; //Last point is inbetween the buffer; redo at the beginning of new buffer.
				break;
			}

			if (i == cloudptr->size()) { 
				cx_increment = 0;
				break;	
			}
				
		}

		fstream.write(buf, cx - cx_increment );
	}
	fstream.close();

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "--> Data export time: " << timer.count() << " seconds\n" << std::endl;
}

//read file into a vector of custom point types
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

