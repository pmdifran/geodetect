
//#define LOG_ALL_OFF
//GeoDetection includes
#include "readascii.h"
#include "GeoDetection.h"
#include "autoRegistration.h"
#include "log.h"

//stdlib includes
#include <iomanip> //for set_precision.

//Command line interfacing with CLI11
#include "CLI/App.hpp"
#include "CLI/Formatter.hpp"
#include "CLI/Config.hpp"

//File handling 
#include <filesystem>

#define SUBSAMPLE_DISTANCE 0.1f
#define NORMAL_RADIUS 1.0f

int main(int argc, char* argv[])
{
	//Initialize logger and start timer.
	geodetection::Log::Init();
	SetConsoleOutputCP(CP_UTF8); //so we can print unicode (needed for progress bars)	
	geodetection::Timer timer;

	// COMMAND LINE INTERFACING.....................................................................................
	if (argc == 1) { GD_ERROR("No arguments passed. Use argument -h or --help for instructions."); return 0; }

	//User-inputted file names
	std::string source_filename, reference_filename, batch_directory;

	//CLI parsing object
	CLI::App app{ "Auto Registration" };

	//Add options (CLI::Option*)
	auto input_opt = app.add_option("-i,--input", source_filename, "Input source cloud filename.");
	auto mask_opt = app.add_option("-r,--reference", reference_filename, "Input reference file (stays constant in batch mode).");
	auto batch_opt = app.add_option("-b,--batch", batch_directory,
		"Input directory containing *only* input source files (ascii) for batch processing. "
		"Ignores -i if set. Use '-b .' for current directory, or '-b ..' for parent directory.");

	//Set option requirements
	mask_opt->required();
	input_opt->excludes(batch_opt); //input option ignored if a batch location is provided. 

	CLI11_PARSE(app, argc, argv);

	// GEODETECTION .................................................................................................
	GD_TITLE("GeoDetection");

	//Construct AsciiReader
	GeoDetection::AsciiReader reader;

	//If batch file mode is on, batch process!
	if (*batch_opt)
	{
		//Import reference as a Registration Cloud and downsample
		GD_TRACE("Reference file name: {0}", reference_filename);
		reader.setFilename(reference_filename);

		GeoDetection::Cloud reference = reader.import();
		reference.distanceDownSample(SUBSAMPLE_DISTANCE);

		//Get list of files in directory. 
		const std::filesystem::path dir(batch_directory);

		//Iterate through batch directory files
		for (auto const& file : std::filesystem::directory_iterator{ dir })
		{
			//get file strings
			std::string file_string = file.path().string();
			std::string file_name_string = file.path().stem().string();
			std::string ext_string = file.path().extension().string();

			//skip over binaries, and mask file
			if (ext_string == ".dll") { continue; }
			if (ext_string == ".exe") { continue; }
			if ((file_name_string + ext_string) == reference_filename) { continue; }

			//set output file name
			std::string out_filename = file_name_string + "_Registered" + ext_string;
			std::string out_transformation_filename = file_name_string + "_transformationMAT" + ".DAT";

			//Import source file and distance downsample
			reader.setFilename(file_string);
			GeoDetection::Cloud source = reader.import();
			source.distanceDownSample(SUBSAMPLE_DISTANCE);

			//Auto Register
			GeoDetection::getGlobalRegistration(reference, source, NORMAL_RADIUS);
			GeoDetection::getICPRegistration(reference, source);

			//Export                              //remember - the cloud is subsampled. 
			source.writeAsASCII(out_filename); // If you want to check alignment quality with full res, 
												// --> apply tranformation matrix to full resolution cloud in CloudCompare/ RiSCAN 

			source.writeTransformation(out_transformation_filename);
		}
	}

	else if (*input_opt)
	{
		//get file strings
		std::filesystem::path source_path(source_filename);
		std::string file_string = source_path.string();
		std::string file_name_string = source_path.stem().string();
		std::string ext_string = source_path.extension().string();

		//Import files and downsample
		reader.setFilename(reference_filename);
		GeoDetection::Cloud reference = reader.import();
		reference.distanceDownSample(SUBSAMPLE_DISTANCE);

		reader.setFilename(source_filename);
		GeoDetection::Cloud source = reader.import();
		source.distanceDownSample(SUBSAMPLE_DISTANCE);

		//Auto Register
		GeoDetection::getGlobalRegistration(reference, source, NORMAL_RADIUS);
		GeoDetection::getICPRegistration(reference, source);

		//Output file
		std::string out_filename = file_name_string + "_Classified" + ext_string;
		source.writeAsASCII(out_filename);
	}

	GD_WARN("Total time: {0} s \n", timer.getDuration());
}
