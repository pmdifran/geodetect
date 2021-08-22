# GeoDetection
Tools for processing and extracting information from point clouds of natural environments. Under development, primarily for Windows. 

### Current features:
- Fast c-style ascii import of LiDAR using the fast float library for parsing
- GeoDetection Cloud class 
  - Revolves around the Point Cloud Library (PCL)
  - Methods aid with common point cloud processing objectives, and member variables store critical data structures (i.e. point cloud kd-trees) for reuse throughout object lifetime.
- autoRegistration: module for coarse and icp point-set registration. 
- macro-based logging implementation of spdlog

### To be added...
-  _IO_: Templated c-style ascii IO for custom point clouds with numerous scalar features.
-  _IO_: Implementing LibLAS, LASLIB, or PDAL libary for importing .las pointclouds.
-  _IO_: Implementing RIEGL laser intruments' SDK for .rdb IO
-  _Method_: User-input with viz (i.e. selection camera position for orienting normals [m_view])
-  _Method_: Feature extraction (trees from lidar)
-  _Method_: Segmentation (supervoxels & region growing)
-  _Data struct_: Intuitive template for creating custom pcl point types (and perhaps templated GeoDetection objects).
-  _OVERALL_: Implementation of CUDA-PCL for rapid real-time processing
-  .... .... .... cross platform stuffs


## Windows Build

**Requirements:**
- Git
- CMake

1. Make a folder (e.g. C:/Dev)

2. Clone the GeoDetection repository into your folder (cloned into a "GeoDetection" folder).
```
$ cd C:/Dev
$ git clone https://github.com/pmdifran/GeoDetection.git
```

3. Download the PCL-AllInOne installer from the PCL releases on github (i.e. `PCL-1.12.0-AllInOne-msvc2019-win64.exe`).

4. Download the PDB files from PCL releases (i.e. `pcl-1.12.0-rc1-pdb-msvc2019-win64.zip`).\
--> Allows you to debug inside the PCL libraries.

5. Launch the installer, and install it into your folder (e.g. C:/Dev/PCL<version>). Copy the .pdb files into PCL/bin.

6. Download PDAL using conda package manager:\
--> Create dependencies directory with `pdal` folder (e.g. C:/Dev/dependencies/pdal)\
-->install pdal in this directory: `conda create --yes --channel conda-forge pdal -p C:/Dev/dependencies/pdal`

7. Build the project with CMake from the command line.
```
cd C:/Dev
mkdir build
cd build
cmake -Ddependencies_DIR=C:/Dev/dependencies ../GeoDetection
```

8. Open the build file created for MSVS (i.e. `C:/Dev/Build/GeoDetection.sln`)

## Linux; MacOS; Windows (custom)

**Requirements:** \
*Look at PCL documentation for minimum required versions*
- Git
- CMake
- Boost (download and build with cmake)
- FLANN (github release download and build with cmake)
- qhull (download and build)
- Eigen (header only --> download)
- VTK
- Qt is neccessary for some visualization libraries if you want to use them.\
--> Qt itself is huge - double check which parts of Qt are required.\

1. Point CMake to the dependencies. Check out PCL build guides for details @ www.pointclouds.org
2. Configure and Build

The GeoDetection CMakeLists.txt is not yet written for the custom build. In the project file you will have to:\
--> Add includes\
--> Add library dependency folder\
--> Add .lib to linker inputs.\
--> Build to the same bin folder as pcl, or write some post-build events to copy over the .dll's (or unix-like equivalents)\
--> Configure OpenMP (/openmp)
