#include "readascii_core.h"

//HELPER FUNCTIONS
size_t
getNumColumns(const std::string& fname)
{
	size_t num_columns = 0;

	//input filestream
	std::ifstream in(fname);

	//objects for parsing
	float token = 0;
	std::string line; //string object for getline

	if (!in.is_open()) {
		GD_ERROR("Failed to open file: {0} \n", fname);
		std::exit(EXIT_FAILURE);
	}

	std::getline(in, line);
	std::getline(in, line); //take second line incase of header
	std::istringstream iss(line);
	iss.imbue(std::locale(std::locale(), new csv_reader())); //treat CSV delimiters as whitespace

	while (iss >> token) num_columns++;

	in.close();
	return num_columns;
}

void
checkIfXYZ(const std::string& fname)
{
	//Check that there is XYZ data
	size_t num_columns = getNumColumns(fname);
	if (num_columns < 3) {
		GD_ERROR(":: Number of columns detected: {0}\
			\n--> The inputted file must contain XYZ coordinates.", num_columns);
		std::exit(EXIT_FAILURE);
	}
}

uintmax_t
getLineCount(const std::string& fname)
{
	const char* fname_cstr = fname.c_str();
	FILE* file;
	errno_t err;

	if ((err = fopen_s(&file, fname_cstr, "r")) != 0)
	{
		GD_ERROR("Failed to open file: {0} \n", fname_cstr);
	}

	static const auto BUFFER_SIZE = 16 * 1024;
	char buf[BUFFER_SIZE + 1];
	uintmax_t num_lines = 1;

	while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {

		if (bytes_read == std::numeric_limits<size_t>::max())
			GD_ERROR("Read failed \n");
		if (!bytes_read)
			break;

		//memchr searches first x bytes of buffer and returns pointer to the first instance of '\n'. If none found, returns nullptr
		for (char* p = buf; (p = (char*)memchr(p, '\n', (buf + bytes_read) - p)); p++)				// --> nullptr is implictly converted into booleans false, which terminates the loop :)
			num_lines++;								  // ^^^^^^^^^^^^^^^^^ Gets the remaining # of addresses in our buffer
	}

	fclose(file);

	//If last character is newline, reduce the number of lines by 1
	std::ifstream in(fname);

	if (!in.is_open()) {
		GD_ERROR("Failed to open file: {0} \n", fname);
	}

	char ch;
	in.seekg(-1, std::ios_base::end);
	in.get(ch);

	if (ch == '\n') { num_lines--; }

	in.close();
	return num_lines;
}

bool
hasHeader(const std::string& fname)
{
	//input filestream
	std::ifstream in(fname);

	//objects for parsing
	std::vector<float> xyz(3, 0);
	std::string line; //string object for getline

	if (!in.is_open()) {
		GD_ERROR("Failed to open file: {0} \n", fname);
		std::exit(EXIT_FAILURE);
	}

	//Check if the file has a header
	std::getline(in, line);
	std::istringstream iss(line); //construct into stringstream object
	iss.imbue(std::locale(std::locale(), new csv_reader())); //using locale that treats CSV delimiters as whitesapce.

	if (iss >> xyz[0] >> xyz[1] >> xyz[2]) { // stream extraction returns false if a non-numeric value is entered
		GD_TRACE(":: No header detected.");
		in.close();
		return false;
	}
	else {
		GD_TRACE(":: Header found.");
		in.close();
		return true;
	}
}

char
getDelimeter(const std::string& fname)
{
	const char delimeters[] = { ',', ' ', ';', 0 };

	//input filestream
	std::ifstream in(fname);

	//objects for parsing
	float temp = 0;
	std::string line; //string object for getline

	in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
	if (!in.is_open()) {
		GD_ERROR("Failed to open file: {0} \n", fname);
		std::exit(EXIT_FAILURE);
	}

	std::getline(in, line);
	std::getline(in, line); //take second line incase of header

	int i = 0;
	while (i < strlen(delimeters)) {
		if (line.find(delimeters[i]) != std::string::npos)
			break;
		i++;
	}

	if (delimeters[i] == ' ')
	{
		GD_INFO("Delimeter detected: ' '", fname);
	}
	else
	{
		GD_INFO("Delimeter detected: {}", delimeters[i]);
	}

	in.close();
	return delimeters[i];
}



