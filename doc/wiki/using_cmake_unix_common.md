# Using CMake {#md_using_cmake_unix_common}

[TOC]

Here is going to be explained how to setup projects that utilize ACADO. At the moment, this page covers usage of the toolkit from C++, on Linux and OS X operating systems and on Windows while using Cygwin. There are basically two ways to setup a project -- a simple one and one more complex but more powerful procedure.

## A simple way of building ACADO based executables

You can put your own C++ files in the <ACADO_ROOT\>/examples/my_examples folder. The only restriction is that one executable can correspond to only one source (C++) file. After you put your source file in my_examples folder, go to <ACADO_ROOT\>/build and type:
~~~
    cmake ..
    make
~~~
Once the build process is over, return to <ACADO_ROOT\>/examples/my_examples. You will find there your executable, i.e. if the name of your source file is my_nice_code.cpp, you will find there an executable named my_nice_code.

## Building ACADO based executables outside ACADO source tree

### Setting up the environment

The first thing you need to do is to make Linux/OS X environment aware of ACADO toolkit. One way of doing this is to add a simple bash script to your environment file.

* **Linux users and Cygwin users** need to edit .bashrc file in the home folder.
* **OS X users** need to edit .bash_profile (or .profile) file in the home folder. In case you do not have such a file on your computer defined, [here](http://redfinsolutions.com/blog/creating-bashprofile-your-mac) is a short explanation how to create it.

When you open the corresponding file, append the following line to the file:
~~~
    source <ACADO_ROOT>/build/acado_env.sh
~~~
where <ACADO_ROOT\> corresponds to the ACADO toolkit root folder, as explained on the installation instructions [webpage](Installation).

Afterwards, **Linux and Cygwin users** should refresh the environment by typing in the terminal 
~~~
    . ~/.bashrc
~~~
**OS X users** should refresh the environment by typing 
~~~
    . ~/.bash_profile
~~~
or .profile, where applicable.

### Making a simple project

Now we can start setting up a simple project. We will refer to MY_PROJECT folder as to a folder where you want to put source files and build an executable.

* Copy <ACADO_ROOT\>/cmake/FindACADO.cmake to the MY_PROJECT folder.
* Create a file CMakeLists.txt in the MY_PROJECT folder.
* Open CMakeLists.txt in your favorite text editor and copy the following text into it.
~~~~~~


#
# Project settings
#

# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( MY_COOL_PROJECT CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
ADD_EXECUTABLE(        <EXEC_NAME\> <EXEC_SOURCES\> )
TARGET_LINK_LIBRARIES( <EXEC_NAME\> ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( <EXEC_NAME\> PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )


~~~~~~
* Edit <EXEC_NAME\> and <EXEC_SOURCES\> tags. <EXEC_NAME\> tag is the executable name and <EXEC_SOURCES\>is the list of source (.cpp) files. For more help with CMake refer to [CMake website](http://www.cmake.org).
* Go again to the terminal, and navigate to the MY_PROJECT folder. Then type:
~~~
    mkdir build
    cd build
    cmake ..
    make
    cd ..
~~~

When the build process is over, an executable with the name <EXEC_NAME\> should appear in the MY_PROJECT folder.

You can build more than one application using the same set of commands lying below the comment "Build an executable". Please keep in mind that name of the executable, <EXEC_NAME\>, has to be unique.
