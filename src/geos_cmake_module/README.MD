# geos_cmake_module
A package providing FindGEOS.cmake

## Usage
To use the CMake modules provided by this catkin package, you must `<build_depend> `on it in your `package.xml`, like so:

```
<depend>geos_cmake_module</depend>
```

Then you must find_package it in your `CMakeLists.txt` along with your other catkin build dependencies:

```
find_package(catkin REQUIRED 
    COMPONENTS
        ...
        geos_cmake_module
        ...)
```

After the above find_package invocations, the modules provided by cmake_modules will be available in your CMAKE_MODULE_PATH to be found. For example you can find `libgeos` by using the following:

```
find_package(GEOS REQUIRED)
```

To use libgeos, be sure to add the GEOS variables to the following:

```
include_directories(
  ...
  ${GEOS_INCLUDE_DIRS}
)
```

```
target_link_libraries(
  ...
  ${GEOS_LIBRARIES}
)
```
