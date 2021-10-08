#pragma once
#pragma warning(push, 0)
//Macros for displaying an OpenMP thread-safe progress bar, using indicators library.

//indicators - progress bars
#include <indicators/cursor_control.hpp>
#include <indicators/block_progress_bar.hpp>

/*Sets console code page to UTF - 8 so console known how to interpret string data */
/*Enables buffering to prevent VS from chopping up UTF - 8 byte sequences */
/*Must call this before using progress bars.*/
#ifndef PROGRESS_OFF

//Create progress with the symbol: bar_name.
#define GD_PROGRESS(NAME, CLOUDSIZE)																	\
	indicators::show_console_cursor(false);																\
	indicators::BlockProgressBar NAME																	\
{																										\
	indicators::option::BarWidth{50},																	\
	indicators::option::Start{" ["},																	\
	indicators::option::End{"]"},																		\
	indicators::option::PrefixText{"Computing...   "},													\
	indicators::option::ForegroundColor{indicators::Color::white},										\
	indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}		\
};																										\
int64_t NAME ## _count = 0;																				\
int64_t NAME ## _increment = CLOUDSIZE * 0.01; 															\
NAME.set_progress(0.0)

//Increment progress bar (thread safe - must be called after GD_PROGRESS(<bar_name>)
#define GD_PROGRESS_INCREMENT(NAME, CLOUDSIZE)					\
do {															\
	_Pragma("omp atomic")										\
	NAME ## _count++;											\
																\
	if (NAME ## _count == CLOUDSIZE) {NAME.mark_as_completed();}	\
	if ((NAME ## _count) % (NAME ## _increment) == 0)			\
		{														\
			_Pragma("omp critical")								\
			NAME.tick();										\
		}														\
}																\
while(0)
//do... while(0) that this macro can be semicolon terminated

#else
#define GD_PROGRESS(NAME, CLOUDSIZE)
#define GD_PROGRESS_INCREMENT(NAME)	
#endif

#pragma warning(pop)
