# GeoDetection
Tools for processing and extracting information from point clouds of natural environments. Under development. 

Currently only for Windows 10. 
\
\
\
**Current features:**
- Fast c-style ascii import of LiDAR into pcl::pointcloud objects (PCLreadASCII)
- GeoDetection class 
  - Revolves around the Point Cloud Library (PCL)
  - Methods aid with common point cloud processing objectives, and member variables store critical data structures for reuse throughout object lifetime.
- autoRegistration: functions for automatic point set registration utilizing GeoDetection's not-so-abstract-methods :)
\
\
\
**Working towards:**
-  _IO_: Templated c-style ascii IO for custom point clouds with numerous scalar features.
-  _IO_: Implenting LibLAS libary for importing .las pointclouds.
-  _IO_: Implementing RIEGL laser intruments' SDK for IO
-  _Method_: User-input with viz (i.e. selection camera position for orienting normals [m_view])
-  _Method_: Feature extraction (trees from lidar)
-  _Method_: Segmentation (supervoxels & region growing)
-  _Data struct_: Intuitive template for creating custom pcl point types (and perhaps templated GeoDetection objects).
-  _OVERALL_: Implementation of CUDA-PCL
-  .... .... .... cross platform stuffs


## Build
### Windows
I do not want to learn how to use cmake - but at some point I'll do it so that this doesn't suck...

1. Get pcl library using vcpkg and integrate the library with MSVS:
```
vcpkg clone https://github.com/microsoft/vcpkg
cd vcpkg
bootstrap-vcpkg.bat
vcpkg install pcl:x64-windows
vcpkg integrate install
```
**OR** integrate project only with:
```
vcpkg integrate project
```
   --> and then use the NuGet package manager to integrate vcpkg with individual MSVS project. 

2. Add source files to a MSVS project:
   - Properties --> Configuration Properties --> C/C++ --> General --> SDL checks: **No**
   - Properties --> Configuration Properties --> C/C++ --> Language --> Open MP support: **Yes(/openmp)**


