# Installation with CMake build system in OS X {#md_installation_cmake_os_x}

[TOC]

Here is explained how to download, compile and use ACADO toolkit with [CMake](http://www.cmake.org) build system in OS X. 

## Prerequisites {#md_iosx_pre}

You can obtain a set of native developer tools for OS X platform by installing _Command Line Tools_ which can be downloaded from the [Apple Developer website](http://developer.apple.com/resources/). There you should go to "OS X" -> "Downloads". You will be asked at this point for your Apple ID (registration is for free). You should download now _Command Line Tools_ for your version of OS X, i.e. for Lion one should download _Command Line Tools (OS X Lion)_ version.

One way to install [CMake](http://www.cmake.org), [Gnuplot](http://gnuplot.info/) and [Doxygen](http://www.doxygen.org) is via [MacPorts](http://www.macports.org/). Please read this [webpage](http://www.macports.org/install.php) (under "Mac OS X Package (.pkg) Installer") to find out how to install MacPorts. After you install MacPorts, you can install those packeges by typing the following command in a terminal (you will need root privileges):
~~~
sudo port install cmake gnuplot doxygen graphviz
~~~
> **NOTE** Gnuplot and Doxygen are optional. ACADO can work without Gnuplot and/or Doxygen, but in this case you will not be able to visualize results and/or generate API documentation.

## Installation {#md_iosx_inst}

Please proceed to [this](@ref md_installation_cmake_unix_common) webpage for further instructions. 
