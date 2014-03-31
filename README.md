# Ground Truth System

Ground Truth System - A tool for visually tracking a moving target on a calibrated ground plane and recording position and angle in 2-dimensions.

[![Build Status](https://travis-ci.org/dysonltd/gts.png?branch=develop)](https://travis-ci.org/dysonltd/gts)

![Screenshot](help/doc/gts_userguide_files/screenshot.png?raw=true)

Additional functionality is included to facilitate the generation of [IEC](http://www.iec.ch/) specific results.

## Requirements

### Linux

__CMake__ (2.8.10.1)
	
	sudo apt-get install cmake

__Qt__ (4.8.1)
	
	sudo apt-get install libqt4-dev qt4-dev-tools
	
__Unicap__ (0.9.12)

	sudo apt-get install libunicap2-dev

__OpenCV__ (2.4.6)

	git clone https://github.com/Itseez/opencv.git 
	git checkout 2.4.6
	mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/opencv/install
	cmake --build . --target install



### Windows

* [CMake](http://www.cmake.org/cmake/resources/software.html) (2.8.10.1)

* [Xvid](http://www.xvid.org/) (1.3.2)

* [OpenCV](http://sourceforge.net/projects/opencvlibrary/files/opencv-win/) (2.4.6)

__MinGW Only__

* [MinGW](ftp://ftp.qt.nokia.com/misc/MinGW-gcc440_1.zip) (4.4)

* [Qt MinGW Libraries](http://qt-project.org/downloads) (4.8.3)

* [MSYS](http://www.mingw.org/wiki/MSYS) (1.0.11)

__Visual Studio Only__

* [Microsoft Visual C++ 2010](http://www.microsoft.com/visualstudio/eng/products/visual-studio-2010-express)

* [Qt Visual Studio 2010 Libraries](http://download.qt-project.org/official_releases/qt/4.8/4.8.5/qt-win-opensource-4.8.5-vs2010.exe)(4.8.3)

__Installer__

* [NSIS (Nullsoft Scriptable Install System)](http://nsis.sourceforge.net/)

## Compiling

From root directory:

__Linux__

	$ mkdir build && cd build
	$ cmake [options] ../
	    e.g. cmake -DOpenCV_ROOT_DIR=/path/to/opencv/install -DCMAKE_BUILD_TYPE=Debug -DGTS_TESTS=ON -DCMAKE_INSTALL_PREFIX=/home/username/gts/ ../
	$ cmake --build . --target install

__Windows (MinGW)__

	mkdir build
	cd build
	cmake.exe -G"MinGW Makefiles" -DOpenCV_ROOT_DIR=C:\PATH-TO-OPENCV-INSTALL ..   
	cmake --build . --target install

Alternatively, call [scripts/minGW_app.bat](scripts/minGW_app.bat) from root directory.

__Windows (VS2010)__

	mkdir build
	cd build
	cmake.exe -G"NMake Makefiles" -DOpenCV_ROOT_DIR=C:\PATH-TO-OPENCV-INSTALL ..
	cmake --build . --target install
	
Alternatively, call [scripts/vs2010_app.bat](scripts/vs2010_app.bat) from root directory.

__CMake Options__

- `OpenCV_ROOT_DIR` - Path to OpenCV libraries and includes (REQUIRED on Windows, looks in standard places on Linux)
- `GTS_TESTS` - Build Google Test Unit Tests (default=OFF).
- `GTS_HELP` - Build GTS Help Assistant (default=ON).
- `CMAKE_INSTALL_PREFIX` - Location for `make install` to place binary and help files.  

##Windows Installer

To redistribute this application on Windows, an installer is available. You will first need to download the 2 additional files (Xvid and Microsoft VS2010 Redist package as linked in the [DOWNLOADS](/installer/files/DOWNLOADS) file.

__MinGW__
	
	mkdir build
	cd build
	cmake.exe -G"MinGW Makefiles" -DOpenCV_ROOT_DIR=C:\PATH-TO-OPENCV-INSTALL ..
	mingw32-make package

Alternatively, call [scripts/minGW_installer.bat](scripts/minGW_installer.bat) from root directory.

__VS2010__
  	
	mkdir build
	cd build
	cmake.exe -G"NMake Makefiles" -DOpenCV_ROOT_DIR=C:\PATH-TO-OPENCV-INSTALL .. 
	nmake package
	
Alternatively, call [scripts/vs2010_installer.bat](scripts/vs2010_installer.bat) from root directory.

## Unit Tests

GTS uses the Google Test Framework for unit testing. A few tests have been included, but we hope to add to these over time and welcome any additions.

## Supported Cameras

The Unicap library on Linux and DirectShow on Windows are used to enumerate cameras. OpenCV is used for recording and tracking of videos.
Therefore cameras that are supported by these libraries should work on the GTS.

## Issues

Bugs and feature requests should be added to the [Issues](https://github.com/dysonltd/gts/issues) section of this repository. If you have a fix for such, please see below to have it considered to be merged in.

## Contributions

For contributions to be considered we require that users first read and follow the steps in the [CONTRIBUTING](CONTRIBUTING.md) file.

## Documentation 

In addition to the __help__ tool that can be compiled alongside the app, the documentation can be found in the [Wiki](https://github.com/dysonltd/gts/wiki).

## License

This application is distributed under the GPLv3 license found in the [LICENSE](LICENSE) file.

Attributions and licenses for third-party libraries can be found in the [OPEN SOURCE LICENSES](OPENSOURCE_LICENSES) file.
