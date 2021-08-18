#include "log.h"

namespace GeoDetection
{
	std::shared_ptr<spdlog::logger> Log::s_core_logger;
	std::shared_ptr<spdlog::logger> Log::s_client_logger;

	void Log::Init()
	{			 //timestamp, name of logger, our message (from cherno)
		spdlog::set_pattern("%^[%T] %v%$");

		s_core_logger = spdlog::stdout_color_mt("Core");
		s_core_logger->set_level(spdlog::level::trace);

		s_client_logger = spdlog::stdout_color_mt("App");
		s_client_logger->set_level(spdlog::level::trace);

	}
}
