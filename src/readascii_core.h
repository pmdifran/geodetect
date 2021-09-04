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

size_t getNumColumns(const char* fname);

void checkIfXYZ(char const* fname);

uintmax_t getLineCount(char const* fname);

bool hasHeader(char const* fname);

char getDelimeter(const char* fname);

//Locale to treat commas, spaces, and newlines as whitespace. 
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
