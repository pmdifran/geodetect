#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

//add to core include
#include <memory>


namespace GeoDetection
{
	class Log
	{
	public:
		static void Init();

		inline static std::shared_ptr<spdlog::logger>& getCoreLogger() { return s_core_logger; }
		inline static std::shared_ptr<spdlog::logger>& getClientLogger() { return s_client_logger; }

	private: 
		static std::shared_ptr<spdlog::logger> s_core_logger;
		static std::shared_ptr<spdlog::logger> s_client_logger;

	};

}