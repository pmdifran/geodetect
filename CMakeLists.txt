cmake_minimum_required( VERSION 3.0 )

set(GeoDetection_source ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(GeoDetection_vendor ${CMAKE_CURRENT_SOURCE_DIR}/vendor) 
set(CMAKE_CXX_STANDARD 23)

# Create Project
project( GeoDetection LANGUAGES CXX)

#Set Geodetecton source files
set(SOURCE
    ${SOURCE}
    ${GeoDetection_source}/main.cpp
    ${GeoDetection_source}/autoRegistration.cpp
    ${GeoDetection_source}/GeoDetection.cpp
    ${GeoDetection_source}/log.cpp
    ${GeoDetection_source}/segmentVegetation.cpp
    ${GeoDetection_source}/ScalarField.cpp
    ${GeoDetection_source}/features.cpp
    ${GeoDetection_source}/readascii_core.cpp
    ${GeoDetection_source}/readascii.cpp
    ${GeoDetection_source}/maskclassify.cpp
    ${GeoDetection_source}/core.cpp
)

set(HEADERS
    ${HEADERS}
    ${GeoDetection_source}/autoRegistration.h
    ${GeoDetection_source}/GeoDetection.h
    ${GeoDetection_source}/log.h
    ${GeoDetection_source}/progressbar.h
    ${GeoDetection_source}/segmentVegetation.h
    ${GeoDetection_source}/ScalarField.h
    ${GeoDetection_source}/features.h
    ${GeoDetection_source}/readascii_core.h
    ${GeoDetection_source}/readascii.h
    ${GeoDetection_source}/maskclassify.h
    ${GeoDetection_source}/core.h
    ${GeoDetection_source}/core_impl.h
    ${GeoDetection_source}/registration_cloud.h
)

#Create target
add_executable( GeoDetection ${SOURCE} ${HEADERS} )
include_directories(${GeoDetection_source})

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

#PDAL
#######################################################################################
if (NOT DEFINED dependencies_DIR)
    set(dependencies_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies)
endif()

# Find PDAL - in development
set(PDAL_DIR ${dependencies_DIR}/pdal/Library/lib/cmake/PDAL)
find_package(PDAL 1.0.0 CONFIG)

if (PDAL_FOUND)
    message("\nPDAL FOUND\n")
    include_directories(${PDAL_INCLUDE_DIRS})
    link_directories(${PDAL_LIBRARY_DIRS})
    add_definitions(${PDAL_DEFINITIONS}) 
##This flag is not compatible with AVX, so FPFH will be prone to heap corruption... Hopefully pdal works without
endif()

target_link_libraries( GeoDetection PRIVATE ${PDAL_LIBRARIES})

#PCL
#######################################################################################
# Find PCL (from default PCL cmake implementation)
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

#Include GeoDetection 3rdPartys (via git submodules)
#######################################################################################
#spdlog
include_directories(${GeoDetection_vendor}/spdlog/include)

#fast_float
include_directories(${GeoDetection_vendor}/fast_float/include)

#CLI11
include_directories(${GeoDetection_vendor}/CLI11/include)

#indicators (for progress bar)
include_directories(${GeoDetection_vendor}/indicators/include)

#MSVS output directories
#######################################################################################
#Set output directories (to coincide with PCL libraries and binaries - i.e. "../PCL 1.12/bin/")
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
	
    #set directory to where test_data is. 
    set_property(TARGET GeoDetection PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test_data)

endif ( MSVC )

#Enable Advanced Vector Extensions (AVX) (otherwise some PCL features might cause heap corruption)
#######################################################################################

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_OPT_ARCH_NATIVE_SUPPORTED)

if (COMPILER_OPT_ARCH_NATIVE_SUPPORTED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
else()
  CHECK_CXX_COMPILER_FLAG("/arch:AVX" COMPILER_OPT_ARCH_AVX_SUPPORTED)
    if(COMPILER_OPT_ARCH_AVX_SUPPORTED)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX")
    endif()
endif()