#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

//add to core include
#include <memory>
#include <chrono>


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

	class Time
	{	
	public:
		Time() {}

		inline static std::chrono::time_point<std::chrono::steady_clock> getStart() 
		{ return std::chrono::steady_clock::now(); }

		inline static double getDuration(std::chrono::time_point<std::chrono::steady_clock>& start_time)
		{
			return std::chrono::duration<double, std::ratio<1, 1000>>
				(std::chrono::steady_clock::now() - start_time).count();
		}
	};
}

//Preprocessors to include or ignore log functions.
#if !defined LOG_CORE_OFF || !defined LOG_ALL_OFF 
#define GD_CORE_TRACE(...)     ::GeoDetection::Log::getCoreLogger()->trace(__VA_ARGS__)
#define GD_CORE_INFO(...)      ::GeoDetection::Log::getCoreLogger()->info(__VA_ARGS__)
#define GD_CORE_WARN(...)      ::GeoDetection::Log::getCoreLogger()->warn(__VA_ARGS__)
#define GD_CORE_ERROR(...)     ::GeoDetection::Log::getCoreLogger()->error(__VA_ARGS__)
#define GD_CORE_FATAL(...)     ::GeoDetection::Log::getCoreLogger()->critical(__VA_ARGS__)

#else
#define GD_CORE_TRACE
#define GD_CORE_INFO
#define GD_CORE_WARN
#define GD_CORE_ERROR
#define GD_CORE_FATAL
#endif

#if !defined LOG_CLIENT_OFF ||  !defined LOG_ALL_OFF
#define GD_TRACE(...)          ::GeoDetection::Log::getClientLogger()->trace(__VA_ARGS__)
#define GD_INFO(...)           ::GeoDetection::Log::getClientLogger()->info(__VA_ARGS__)
#define GD_WARN(...)           ::GeoDetection::Log::getClientLogger()->warn(__VA_ARGS__)
#define GD_ERROR(...)          ::GeoDetection::Log::getClientLogger()->error(__VA_ARGS__)
#define GD_FATAL(...)          ::GeoDetection::Log::getClientLogger()->critical(__VA_ARGS__)

#else
#define GD_TRACE
#define GD_INFO
#define GD_WARN
#define GD_ERROR
#define GD_FATAL
#endif



