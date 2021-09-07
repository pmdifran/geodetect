
//#define LOG_ALL_OFF
//GeoDetection includes
#include "GeoDetection.h"
#include "readascii.h"
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
		"Input directory containing *only* input source files (ascii) for batch processing. "
		"Ignores -i if set. Use '-b .' for current directory, or '-b ..' for parent directory.");

	//Set option requirements
	mask_opt->required();
	input_opt->excludes(batch_opt); //input option ignored if a batch location is provided. 

	CLI11_PARSE(app, argc, argv);
	
	// GEODETECTION .................................................................................................
	//Initialize logger and start timer.
	GeoDetection::Log::Init();
	auto start = GeoDetection::Time::getStart();
	GD_TITLE("GeoDetection");
	
	//Construct AsciiReader
	GeoDetection::AsciiReader reader; 

	//Import mask
	GD_TRACE("Mask file name: {0}", mask_filename);
	reader.setFilename(mask_filename);
	GeoDetection::Cloud mask = reader.import();
	
	//if batch file mode is on, batch process!
	if (*batch_opt)
	{
		//Get list of files in directory. 
		const std::filesystem::path dir(batch_directory);

		for (auto const& file : std::filesystem::directory_iterator{ dir })
		{
			//get file strings
			std::string file_string = file.path().string();
			std::string file_name_string = file.path().stem().string();
			std::string ext_string = file.path().extension().string();

			//skip over binaries, and mask file
			if (ext_string == ".dll") { continue; }
			if (ext_string == ".exe") { continue; }
			GD_TRACE("File string: {0}  |   Mask filename: {1}", file_string, mask_filename);
			if ((file_name_string + ext_string) == mask_filename) { continue; }

			//set output file name
			std::string out_filename = file_name_string + "_Classified" + ext_string;

			//Import file
			reader.setFilename(file_string);
			GeoDetection::Cloud source = reader.import();

			//Classify
			GeoDetection::classifyClusters(mask, source, 7);

			//Export
			source.writeAsASCII(out_filename);
		}
		
	}

	else if (*input_opt)
	{
		//get file strings
		std::filesystem::path source_path(source_filename);
		std::string file_string = source_path.string();
		std::string file_name_string = source_path.stem().string();
		std::string ext_string = source_path.extension().string();

		//Import file
		reader.setFilename(source_filename);
		GeoDetection::Cloud source = reader.import();

		//Classify
		GeoDetection::classifyClusters(mask, source, 7);

		//Output file
		std::string out_filename = file_name_string + "_Classified" + ext_string;
		source.writeAsASCII(out_filename);
	}

	GD_WARN("Total time: {0} s \n", (GeoDetection::Time::getDuration(start)/1000));
}
