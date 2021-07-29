cmake_minimum_required( VERSION 3.0 )

# Create Project
project( GeoDetection LANGUAGES CXX)

#Set Geodetecton source files
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/autoRegistration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeoDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PCLreadASCII.cpp
)

set(HEADERS
    ${HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/autoRegistration.h
    ${CMAKE_CURRENT_SOURCE_DIR}/fast_float.h
    ${CMAKE_CURRENT_SOURCE_DIR}/GeoDetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/PCLreadASCII.h
)

add_executable( GeoDetection ${SOURCE} ${HEADERS} )

#Append 'd' to debug builds.
set_target_properties(GeoDetection PROPERTIES DEBUG_POSTFIX d)

# Set StartUp Project (Option)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "GeoDetection" )

#Set OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(GeoDetection PUBLIC "${OpenMP_CXX_FLAGS}")
    target_compile_options(GeoDetection PUBLIC "${OpenMP_CXX_FLAGS}")
endif()

# Find Packages
find_package( PCL 1.12 REQUIRED )

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
  target_link_libraries( GeoDetection PUBLIC ${PCL_LIBRARIES} )
endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src)

#Set output directories (to coincide with PCL libraries and binaries)
if ( MSVC )
    message([STATUS] "MSVS Detected... Setting output directories...")
    set_target_properties( GeoDetection PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PCL_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PCL_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PCL_DIR}/../lib)

    set_target_properties( GeoDetection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PCL_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PCL_DIR}/../lib)
    set_target_properties( GeoDetection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PCL_DIR}/../lib)

    set_target_properties( GeoDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PCL_DIR}/../bin)
    set_target_properties( GeoDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PCL_DIR}/../bin)
    set_target_properties( GeoDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PCL_DIR}/../bin)

    set_property(TARGET GeoDetection PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${PCL_DIR}/../bin)

endif ( MSVC )