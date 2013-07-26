# Installation with CMake build system in Linux {#md_installation_cmake_linux}

[TOC]

Here is explained how to download, compile and use ACADO toolkit with [CMake](http://www.cmake.org) build system in Linux operating system. In particular, the focus is on Ubuntu distribution (or any Debian based distribution).

## Prerequisites {#md_il_pre} 

For starters, one needs to download a few packages via apt-get package manager (you will need root privileges):
~~~
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
~~~
> **NOTE** The following packages are mandatory: gcc, g++, cmake and subversion. [Gnuplot](http://gnuplot.info/), [Doxygen](http://www.doxygen.org) and [Graphviz](http://www.graphviz.org) are optional. ACADO can work without those packages, but you will not be able to visualize results and/or generate API documentation.

## Installation {#md_il_inst}

Please proceed to [this](@ref md_installation_cmake_unix_common) webpage for further instructions.

