
//#define LOG_ALL_OFF
//GeoDetection includes
#include "readascii.h"
#include "GeoDetection.h"
#include "segmentVegetation.h"
#include "log.h"

//stdlib includes
#include <iomanip> //for set_precision.

//testing includes
#include "features.h"

//Command line interfacing with CLI11
#include "CLI/App.hpp"
#include "CLI/Formatter.hpp"
#include "CLI/Config.hpp"

//File handling 
#include <filesystem>

int main(int argc, char* argv[])
{
	//Initialize logger and start timer.
	GeoDetection::Log::Init();
	auto start = GeoDetection::Time::getStart();

	// COMMAND LINE INTERFACING.....................................................................................
	if (argc == 1) { GD_ERROR("No arguments passed. Use argument -h or --help for instructions."); return 0; }

	//User-inputs
	std::string source_filename, batch_directory;
	float num_neighbors_normals = 25;

	//CLI parsing object
	CLI::App app{ "Auto Registration" };

	//Add options (CLI::Option*)
	auto input_opt = app.add_option("-i,--input", source_filename, "Input source cloud filename.");
	auto neighbors_opt = app.add_option("-n,--neighbors", num_neighbors_normals, "Input number of neighbors for normals.");
	auto batch_opt = app.add_option("-b,--batch", batch_directory,
		"Input directory containing *only* input source files (ascii) for batch processing. "
		"Ignores -i if set. Use '-b .' for current directory, or '-b ..' for parent directory.");

	//Set option requirements
	input_opt->excludes(batch_opt); //input option ignored if a batch location is provided. 
	CLI11_PARSE(app, argc, argv);

	// PROCESSING.. .................................................................................................
	GD_TITLE("GeoDetection");

	//Construct AsciiReader
	GeoDetection::AsciiReader reader;

	//If batch file mode is on, batch process!
	if (*batch_opt)
	{
		//Get list of files in directory. 
		const std::filesystem::path dir(batch_directory);

		//Iterate through batch directory files
		for (auto const& file : std::filesystem::directory_iterator{ dir })
		{
			if (file.is_directory()) { continue; }

			//get file strings
			std::string file_string = file.path().string();
			std::string file_name_string = file.path().stem().string();
			std::string ext_string = file.path().extension().string();

			//skip over binaries, and mask file
			if (ext_string == ".dll") { continue; }
			if (ext_string == ".exe") { continue; }

			//set output file name
			std::string out_filename = file_name_string + "_vegetationSegmented" + ext_string;

			//Import source file and distance downsample
			reader.setFilename(file_string);
			GeoDetection::Cloud source = reader.import();

			//Segment vegetation
			GeoDetection::segmentVegetation(source);

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

		//Import files
		reader.setFilename(source_filename);
		GeoDetection::Cloud source = reader.import();

		//testing curvature
		auto normals = source.getNormalsKSearchDemeaned(200);
		GeoDetection::ScalarField curvatures = GeoDetection::NormalsToCurvature(normals);
		source.setNormals(normals);
		source.addScalarField(std::move(curvatures));

		//Output file
		std::string out_filename = file_name_string + "_test_output" + ext_string;
		source.writeAsASCII(out_filename);
	}

	GD_WARN("Total time: {0} s \n", (GeoDetection::Time::getDuration(start) / 1000));
}
