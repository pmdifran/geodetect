# GeoDetection
Tools for processing and extracting information from point clouds of natural environments. Under development, tested for Windows. 

### Current features:
- Macro-ized logging using spdlog.
- Fast c-style ascii import of LiDAR.
- Command line interfacing for single and batch processing, using CLI11.

- GeoDetection Cloud objects
  - A wrapper around the Point Cloud Library cloud: pcl::PointCloud<pcl::PointXYZ>
  - Contains core tools for point cloud operations
  - Provides storage for frequently used pcl types (i.e KdTrees, normals)
  - CloudCompare-esque scalar fields

- Current GeoDetection modules:
  - Auto Registration (global and icp)
  - Vegetation segmentation
  - Mask classification

### To be added...
-  _IO_: Implementing PDAL for importing .las and .laz files.
-  _IO_: Implementing RIEGL laser intruments' SDK for .rdb IO
-  _Method_: User-input with viz (i.e. selection camera position for orienting normals [m_view])
-  _Method_: Segmentation (supervoxels; region growing; object-based segmentation)

-  _OVERALL_: Implementation of CUDA-PCL for rapid real-time processing
-  Ensuring this is cross platform at some point

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

Refer to https://pcl.readthedocs.io/projects/tutorials/en/latest/# for building yourself on Windows 10, or on POSIX compliant systems.
