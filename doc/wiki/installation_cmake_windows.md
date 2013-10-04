# Installation with CMake build system in Windows {#md_installation_cmake_windows}

[TOC]

Here is explained how to download, compile and use ACADO toolkit with [CMake](http://www.cmake.org) build system in Windows. The preferred way is to use the Cygwin based installation.

## Installation for usage within Cygwin {#md_iwin_cyg}

### Prerequisites {#md_iwin_cyg_pre}

*	Go to the [Cygwin](http://www.cygwin.com) website, download and execute the setup program (setup.exe). This may require administrator priveleges.
*	Follow the instructions of the installation program. Make sure that you chose an installation path without any spaces (e.g. c:/cygwin).
*	In "select packages" step, select at least the following packages for installation:
    * *Devel* category: binutils, cmake, (doxygen), gcc4, gcc4-core, gcc4-g++, make, git.
    * *Graphics* category: gnuplot.
    * *X11* category: xinit.
*	Click next and wait for the automatic installation.
*	Start cygwin (a linux terminal will open). To use plotting, you need to open a terminal with the X-server. You can do this directly by running:
	
		C:\cygwin\bin\run.exe /usr/bin/bash.exe -l -c /usr/bin/startxwin.exe
	
	in "run".
* Type in the terminal:
    
		cp /usr/bin/make.exe /usr/bin/gmake.exe

### Installation {#md_iwin_cyg_inst}

Please proceed to [this](@ref md_installation_cmake_unix_common) webpage for further instructions.

## Installation for usage with Visual Studio {#md_iwin_vs}

> **NOTE** Initial efforts have been made to compile ACADO with Visual Studio to be used from C++. However, our time and man power are limited plus our main development platforms are Linux and OS X. That said, we expect community to contribute here. In case you want to contribute, please make your own fork, develop code and make a pull request on GitHub. Thank you for your understanding!

### Prerequisites {#md_iwin_vs_pre}

* Visual Studio Express Edition can be obtained [here](http://www.microsoft.com/visualstudio/en-us).
* [GIT](http://git-scm.com/) client, called tortoisegit, can be downloaded [here](https://code.google.com/p/tortoisegit/).
* A tool for building the source code, called CMake, can be downloaded from [this](http://www.cmake.org/cmake/resources/software.html) webpage.
* Windows binaries for the visualization program [Gnuplot](http://gnuplot.info/) can be downloaded [here](http://sourceforge.net/projects/gnuplot/files/gnuplot/). Our suggestion is to always download the latest version of a Windows installer (file named gp\<VERSION\>-win32-setup.exe). For example, for the version 4.6, a file 4.6.0/gp460-win32-setup.exe should be downloaded and installed.
* A tool for generation of API documentation, Doxygen, can be download [here](http://www.stack.nl/~dimitri/doxygen/download.html#latestsrc).
> **NOTE** Gnuplot and Doxygen are optional. ACADO can work without Gnuplot and/or Doxygen, but in this case you will not be able to visualize results and/or generate API documentation.

### Installation {#md_iwin_vs_inst}

Now you can download the toolkit code by cloning the GIT repository:

* Open your file manager and make a folder called "ACADOtoolkit".
* Open tortoise git application and fill in the field as described [here](https://code.google.com/p/tortoisegit/wiki/Screenshots#Cloning):
    * Enter the URL for cloning. Our suggestion is to always check out __stable__ branch for the latest bug fixes:

        https://github.com/acado/acado.git

    * Point "Directory" field to the ACADOtoolkit folder.
    * Check the "Branch" field and enter **stable**.
* Click "OK" and wait for the code to be downloaded.

After the checkout, you will need to generate a Visual Studio (VS) solution (a set of VS projects). For this purpose:

* Make a folder called "build" inside "ACADOtoolkit" folder you just created.
* Start CMake application (from the "Start" menu).
* "Where is the source code" field: navigate to the folder "ACADOtoolkit"
* "Where to build the binaries" field: navigate to the folder "ACADOtoolkit/build"
* _Gnuplot related_:
    * By default, CMake will search for a Gnuplot executable in folder "C:/gnuplot/bin/".
    * If your Gnuplot installation is not installed there then you have to enter the absolute path of the executable in the "Value" field corresponding to variable "GNUPLOT_EXECUTABLE_PATH" in the CMake window.
* Click "Configure". You will be asked to select a version of VS that is installed on your computer -- please select exactly the one you have.
    * _Gnuplot related_: If CMake found the Gnuplot executable you should see a line in the log box (at the bottom of the CMake window) stating: "Looking for Gnuplot executable: found.". Otherwise, it will state "... not found.", and in this case return to the previous step.
* Click "Generate". This will trigger generation of a VS solution.

OK, now we have generated a VS solution that will build the ACADO toolkit libraries and examples. It is the time to start building ACADO libraries and examples:

* Please start Visual Studio now (from the "Start" menu).
* Click "File" -> "Open" and select the ACADO.sln file inside "ACADOtoolkit/build" folder.
* Click "Debug" -> "Build solution" and wait until libraries and examples are built.
