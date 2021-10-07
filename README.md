# GeoDetection
Tools for processing and extracting information from point clouds of natural environments. Under development, tested for Windows. 

### Current features:
- Macro-based logging & progress bars
- Fast c-style ascii import of LiDAR XYZ & Scalar Fields.
- Command line interfacing for single and batch processing.

- GeoDetection Cloud objects
  - Contains core tools for point cloud operations
  - Provides storage for frequently used pcl types (i.e KdTree, Octree, normals)
  - Storage of arbitrary number of scalar fields, for feature calculation etc. 

### Current GeoDetection modules:**
**Auto Registration (global feature matching and ICP)
<img src="https://user-images.githubusercontent.com/64287741/135674654-be7a1585-3409-4641-b595-2a7715a090cc.PNG" width="1000" height="275">

**Vegetation Segmentation (multiscale feature calculation)**
<img src="https://user-images.githubusercontent.com/64287741/135675257-96e6b762-a54b-4658-861a-d77bd897a8e2.PNG" width="1000" height="525">

**Point cloud classification using masks**

### To be added...
-  _IO_: Implementing PDAL for importing .las and .laz files.
-  _IO_: Implementing RIEGL laser intruments' SDK for .rdb IO.

-  _Module_: PCL visualization (VTK) that supports user input (i.e. selection camera position for orienting normals, interactive auto registration).
-  _Module_: Segmentation (supervoxels; region growing; object-based segmentation).

-  _OVERALL_: CUDA for point-normal and volumetric feature calculation.
-  _OVERALL_: Cross platform testing.

## Windows Build

**Requirements:**
- Git
- CMake

1. Make a folder (e.g. C:/Dev)

2. Clone the GeoDetection repository into your folder (cloned into a "GeoDetection" folder).\
`$ cd C:/Dev`\
`$ git clone --recurse-submodules -j8 https://github.com/pmdifran/GeoDetection.git`

3. Download the PCL-AllInOne installer from the PCL releases on github (i.e. `PCL-1.12.0-AllInOne-msvc2019-win64.exe`).

4. Download the PDB files from PCL releases (i.e. `pcl-1.12.0-rc1-pdb-msvc2019-win64.zip`).\
--> Allows you to debug inside the PCL libraries.

5. Launch the installer, and install it into your folder (e.g. C:/Dev/PCL<version>). Copy the .pdb files into PCL/bin.

6. *Not required yet* Download PDAL using conda package manager:\
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
