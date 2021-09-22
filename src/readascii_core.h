#pragma once
#include"log.h"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>	//for std::cout
#include <fstream>
#include <iomanip>  //for setprecision()
#include <locale> //for csv reader locale
#include <cstdlib> //for exit()

//Gets number of columns in an ASCII file. 
size_t getNumColumns(const std::string& fname);

//Checks if there is only XYZ data.
void checkIfXYZ(const std::string& fname);

//Gets number of lines in the txt file.
uintmax_t getLineCount(const std::string& fname);

//Check whether the file has a header at the top of the file, describing the fields. Returns true if a header is present.
bool hasHeader(const std::string& fname);

//Returns the delimiter that separates the ASCII fields. 
char getDelimeter(const std::string& fname);

//Locale to treat commas, spaces, and newlines as whitespace --> used for C++ IOstreams. 
struct csv_reader : std::ctype<char>
{
	csv_reader()
		: std::ctype<char>(get_table()) {} //construct csv_reader by calling get_table

	static std::ctype_base::mask const* get_table()
	{
		static std::vector<std::ctype_base::mask> rc(table_size, std::ctype_base::mask());

		//define these characters as whitespace
		rc[','] = std::ctype_base::space;
		rc['\n'] = std::ctype_base::space;
		rc[' '] = std::ctype_base::space;
		rc[';'] = std::ctype_base::space;
		return &rc[0]; //returns ctype mapping table address.
	}
};
