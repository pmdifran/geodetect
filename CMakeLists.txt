cmake_minimum_required( VERSION 3.0 )

set(GEODETECT_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(GEODETECT_VENDOR ${CMAKE_CURRENT_SOURCE_DIR}/vendor) 
set(CMAKE_CXX_STANDARD 23)

# Create Project
project( geodetect LANGUAGES CXX)

#Set geodetect source files
set(SOURCE
    ${SOURCE}
    ${GEODETECT_SOURCE}/main.cpp
    ${GEODETECT_SOURCE}/autoRegistration.cpp
    ${GEODETECT_SOURCE}/geodetect.cpp
    ${GEODETECT_SOURCE}/log.cpp
    ${GEODETECT_SOURCE}/segmentVegetation.cpp
    ${GEODETECT_SOURCE}/ScalarField.cpp
    ${GEODETECT_SOURCE}/features.cpp
    ${GEODETECT_SOURCE}/readascii_core.cpp
    ${GEODETECT_SOURCE}/readascii.cpp
    ${GEODETECT_SOURCE}/maskclassify.cpp
    ${GEODETECT_SOURCE}/core.cpp
)

set(HEADERS
    ${HEADERS}
    ${GEODETECT_SOURCE}/autoRegistration.h
    ${GEODETECT_SOURCE}/geodetect.h
    ${GEODETECT_SOURCE}/log.h
    ${GEODETECT_SOURCE}/progressbar.h
    ${GEODETECT_SOURCE}/segmentVegetation.h
    ${GEODETECT_SOURCE}/ScalarField.h
    ${GEODETECT_SOURCE}/features.h
    ${GEODETECT_SOURCE}/readascii_core.h
    ${GEODETECT_SOURCE}/readascii.h
    ${GEODETECT_SOURCE}/maskclassify.h
    ${GEODETECT_SOURCE}/core.h
    ${GEODETECT_SOURCE}/core_impl.h
    ${GEODETECT_SOURCE}/registration_cloud.h
)

#Create target
add_executable( geodetect ${SOURCE} ${HEADERS} )
include_directories(${GEODETECT_SOURCE})

#Append 'd' to debug builds.
set_target_properties(geodetect PROPERTIES DEBUG_POSTFIX d)

# Set StartUp Project (Option)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "geodetect" )

#Set OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(geodetect PUBLIC "${OpenMP_CXX_FLAGS}")
    target_compile_options(geodetect PUBLIC "${OpenMP_CXX_FLAGS}")
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

target_link_libraries( geodetect PRIVATE ${PDAL_LIBRARIES})

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
  target_link_libraries( geodetect PUBLIC ${PCL_LIBRARIES} )
endif()

#Include geodetect 3rdPartys (via git submodules)
#######################################################################################
#spdlog
include_directories(${GEODETECT_VENDOR}/spdlog/include)

#fast_float
include_directories(${GEODETECT_VENDOR}/fast_float/include)

#CLI11
include_directories(${GEODETECT_VENDOR}/CLI11/include)

#indicators (for progress bar)
include_directories(${GEODETECT_VENDOR}/indicators/include)

#MSVS output directories
#######################################################################################
#Set output directories (to coincide with PCL libraries and binaries - i.e. "../PCL 1.12/bin/")
if ( MSVC )
    message([STATUS] "MSVS Detected... Setting output directories...")
    set_target_properties( geodetect PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PCL_DIR}/../lib)
    set_target_properties( geodetect PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PCL_DIR}/../lib)
    set_target_properties( geodetect PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PCL_DIR}/../lib)

    set_target_properties( geodetect PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PCL_DIR}/../lib)
    set_target_properties( geodetect PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PCL_DIR}/../lib)
    set_target_properties( geodetect PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PCL_DIR}/../lib)

    set_target_properties( geodetect PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PCL_DIR}/../bin)
    set_target_properties( geodetect PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PCL_DIR}/../bin)
    set_target_properties( geodetect PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PCL_DIR}/../bin)
	
    #set directory to where test_data is. 
    set_property(TARGET geodetect PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test_data)

endif ( MSVC )

#Enable Advanced Vector Extensions (AVX)
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