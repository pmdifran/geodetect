cmake_minimum_required( VERSION 3.0 )

# Create Project
project( GeoDetection)

#Create dummy file for cmake and create executable with it
file(WRITE null.cpp "")
add_executable( GeoDetection null.cpp )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "GeoDetection" )

#Set output directories (to coincide with PCL libraries and binaries)
if ( MSVC )
    message([STATUS] "MSVS Detected... Setting output directories...")
    set_target_properties( GeoDetection PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../lib)

    set_target_properties( GeoDetection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../lib)

    set_target_properties( GeoDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
    set_target_properties( GeoDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../bin)
    set_target_properties( GeoDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../bin)

    # etc for the other available configuration types (MinSizeRel, RelWithDebInfo)
endif ( MSVC )

# Find Packages
find_package( PCL 1.8 REQUIRED )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( GeoDetection ${PCL_LIBRARIES} )
endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src)