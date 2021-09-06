
//#define LOG_ALL_OFF
//GeoDetection includes
#include "GeoDetection.h"
#include "readascii.h"
#include "autoRegistration.h"
#include "segmentVegetation.h"
#include "maskclassify.h"
#include "log.h"

//stdlib includes
#include <iomanip> //for set_precision.

//Command line interfacing with CLI11
#include "CLI/App.hpp"
#include "CLI/Formatter.hpp"
#include "CLI/Config.hpp"

//File handling 
#include <filesystem>

int main (int argc, char* argv[])
{
	// COMMAND LINE INTERFACING.....................................................................................
	//User-inputted file names
	std::string source_filename, mask_filename, batch_directory;
	
	//CLI parsing object
	CLI::App app{ "Mask Classification" };

	//Add options (CLI::Option*)
	auto input_opt = app.add_option("-i,--input", source_filename, "Input source cloud filename.");
	auto mask_opt = app.add_option("-m,--mask", mask_filename, "Input mask file (classification as last field)"); //ADD OPTION FOR COLUMN
	auto batch_opt = app.add_option("-b,--batch", batch_directory,
		"Input directory containing *only* input source files (ascii) for batch processing. Ignores -i if set.");

	//Set option requirements
	mask_opt->required();
	input_opt->excludes(batch_opt); //input option ignored if a batch location is provided. 

	CLI11_PARSE(app, argc, argv);
	
	// GEODETECTION .................................................................................................
	//Initialize logger and start timer.
	GeoDetection::Log::Init();
	auto start = GeoDetection::Time::getStart();
	
	//Construct AsciiReader
	GeoDetection::AsciiReader reader; 

	//Import mask
	reader.setFilename(mask_filename);
	GeoDetection::Cloud mask = reader.import();
	
	//if batch file mode is on, batch process!
	if (*batch_opt)
	{
		//Get list of files in directory. 
		const std::filesystem::path dir(batch_directory);

		for (auto const& file : std::filesystem::directory_iterator{ dir })
		{
			std::string file_string = file.path().string();
			reader.setFilename(file_string);
			GeoDetection::Cloud source = reader.import();

			GeoDetection::classify(mask, source, 7);

			//get file extension
			size_t ext_index = file_string.find('.');
			std::string ext_string = file_string.substr(ext_index);

			//********************Do some sort of of extension check earlier on.********************************

			//Create new outfile name
			std::string out_filename = file_string.substr(0, ext_index) + "_Classified" + '.' + ext_string;
			
			source.writeAsASCII(out_filename);
		}
		
	}

	else if (*input_opt)
	{
		reader.setFilename(source_filename);
		GeoDetection::Cloud source = reader.import();

		GeoDetection::classify(mask, source, 7);

		//get file extension
		size_t ext_index = source_filename.find('.');
		std::string ext_string = source_filename.substr(ext_index);

		//********************Do some sort of of extension check earlier on.********************************

		//Create new outfile name
		std::string out_filename = source_filename.substr(0, ext_index) + "_Classified" + '.' + ext_string;

		source.writeAsASCII(out_filename);
	}

	GD_WARN("Total time: {0} s \n", GeoDetection::Time::getDuration(start)/1000);
}
