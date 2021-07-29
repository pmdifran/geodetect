# GeoDetection
Tools for processing and extracting information from point clouds of natural environments. Under development. 

### Current features:
- Fast c-style ascii import of LiDAR into pcl::pointcloud objects (PCLreadASCII)
- GeoDetection class 
  - Revolves around the Point Cloud Library (PCL)
  - Methods aid with common point cloud processing objectives, and member variables store critical data structures for reuse throughout object lifetime.
- autoRegistration: functions for automatic point set registration utilizing GeoDetection's not-so-abstract-methods :)

### To be added...
-  _IO_: Templated c-style ascii IO for custom point clouds with numerous scalar features.
-  _IO_: Implementing LibLAS, LASLIB, or PDAL libary for importing .las pointclouds.
-  _IO_: Implementing RIEGL laser intruments' SDK for .rdb IO
-  _Method_: User-input with viz (i.e. selection camera position for orienting normals [m_view])
-  _Method_: Feature extraction (trees from lidar)
-  _Method_: Segmentation (supervoxels & region growing)
-  _Data struct_: Intuitive template for creating custom pcl point types (and perhaps templated GeoDetection objects).
-  _OVERALL_: Implementation of CUDA-PCL
-  .... .... .... cross platform stuffs


## Windows Build

**Requirements:**
- Git
- CMake

1. Make a folder (e.g. C:/Dev)

2. Make a build subfodler (e.g. C:/Dev/Build)

2. Clone the GeoDetection repository into your repository:  
```
$ cd C:/Dev
$ git clone https://github.com/pmdifran/GeoDetection.git
```

3. Download the PCL-AllInOne installer from the PCL releases on github (i.e. `PCL-1.12.0-AllInOne-msvc2019-win64.exe`)

4. Launch the installer, and install it into your folder (e.g. C:/Dev/PCL<version>)

5. Launch the CMake GUI:\
Where is the source code: `C:/Dev/GeoDetection`\
Where to build the binaries: `C:/Dev/Build`\
--> **Configure** \
--> **Generate**

6. Open the `GeoDetection.sln` MSVS solution file, located in the build directory.

7. (*Optional*) Download the PDB files from PCL releases (i.e. `pcl-1.12.0-rc1-pdb-msvc2019-win64.zip`).\
--> Paste the .pdb files into the pcl bin folder (i.e. `C:/Dev/PCL 1.12.0/bin`)\
--> This will allow you to run the debugger inside the PCL libraries. 

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

1. Point CMake to the dependencies. Check out PCL build guides for details.
2. Configure and Build

The GeoDetection CMakeLists.txt is not yet written for the custom build. In the project file you will have to:\
--> Add includes\
--> Add library dependency folder\
--> Add .lib to linker inputs.\
--> Build to the same bin folder as pcl, or write some post-build events to copy over the .dll's (or unix-like equivalents)\
--> Configure OpenMP (/openmp)