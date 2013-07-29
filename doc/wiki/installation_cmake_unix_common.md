# Installation - common for UNIX systems {#md_installation_cmake_unix_common}

Please download the toolkit code. For this purposes you will have to use a
terminal application. Our suggestion is to always clone the __stable__ branch:
~~~
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
~~~
> **NOTE** We will refer later to the ACADOtoolkit folder as <ACADO_ROOT\>.

Go to ACADOtoolkit folder and create a build folder for an out-of-source build:
~~~
cd ACADOtoolkit
mkdir build
cd build
~~~

Run CMake to generate makefiles and start the building process:
~~~
cmake ..
make
~~~

Check whether the installation was successful by running an example:
~~~
cd ..
cd examples/getting_started
./simple_ocp
~~~

Now you can proceed to the [documentation](@ref md_documentation) related page,
where you can find out more about tutorials, source code documentation etc.

## Additonal

In case you want to compile ACADO in debug mode, you can run CMake like this:
~~~
cmake -DCMAKE_BUILD_TYPE=Debug ..
~~~
