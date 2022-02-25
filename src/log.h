#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

//add to core include
#include <memory>
#include <chrono>


namespace geodetect
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

	class Timer
	{	
	public:
		Timer() { setStartToNow(); }

		inline void setStartToNow() { m_start_time = std::chrono::steady_clock::now(); }

		inline double getDuration()
		{
			return std::chrono::duration<double, std::ratio<1, 1000>>
				(std::chrono::steady_clock::now() - m_start_time).count();
		}
	
	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> m_start_time;
	};
}

//Preprocessors to include or ignore log functions.
#if !defined LOG_CLIENT_OFF ||  !defined LOG_ALL_OFF
#define GD_TRACE(...)          ::geodetect::Log::getClientLogger()->trace(__VA_ARGS__)
#define GD_INFO(...)           ::geodetect::Log::getClientLogger()->info(__VA_ARGS__)
#define GD_WARN(...)           ::geodetect::Log::getClientLogger()->warn(__VA_ARGS__)
#define GD_ERROR(...)          ::geodetect::Log::getClientLogger()->error(__VA_ARGS__)
#define GD_FATAL(...)          ::geodetect::Log::getClientLogger()->critical(__VA_ARGS__)

#define GD_TITLE(...)	{                                                                   \
							int num_dashes = 75;                                            \
							std::cout << '\n' << std::endl;                                 \
							GD_WARN(std::string(num_dashes,'-'));                           \
							std::string s = std::string(__VA_ARGS__);                       \
							std::string substr = s.substr(0, s.find(','));                  \
							int length = substr.length();                                   \
							auto new_s = std::string((num_dashes-length)/2,' ') + s;        \
							GD_WARN(new_s);                                                 \
							GD_WARN(std::string(num_dashes,'-'));                           \
						} 

#else
#define GD_TRACE
#define GD_INFO
#define GD_WARN
#define GD_ERROR
#define GD_FATAL
#endif

#if !defined LOG_CORE_OFF || !defined LOG_ALL_OFF 
#define GD_CORE_TRACE(...)     ::geodetect::Log::getCoreLogger()->trace(__VA_ARGS__)
#define GD_CORE_INFO(...)      ::geodetect::Log::getCoreLogger()->info(__VA_ARGS__)
#define GD_CORE_WARN(...)      ::geodetect::Log::getCoreLogger()->warn(__VA_ARGS__)
#define GD_CORE_ERROR(...)     ::geodetect::Log::getCoreLogger()->error(__VA_ARGS__)
#define GD_CORE_FATAL(...)     ::geodetect::Log::getCoreLogger()->critical(__VA_ARGS__)

#define GD_CORE_TITLE(...)	{                                                               \
							int num_dashes = 75;                                            \
							std::cout << '\n' << std::endl;                                 \
							GD_CORE_WARN(std::string(num_dashes,'-'));                      \
							std::string s = std::string(__VA_ARGS__);                       \
							std::string substr = s.substr(0, s.find(','));                  \
							int length = substr.length();                                   \
							auto new_s = std::string((num_dashes-length)/2,' ') + s;        \
							GD_CORE_WARN(new_s);                                            \
							GD_CORE_WARN(std::string(num_dashes,'-'));                      \
						} 

#else
#define GD_CORE_TRACE
#define GD_CORE_INFO
#define GD_CORE_WARN
#define GD_CORE_ERROR
#define GD_CORE_FATAL
#endif

