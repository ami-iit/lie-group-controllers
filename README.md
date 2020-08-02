<p align="center">
<h1 align="center">lie-group-controllers </h1>
</p>

<p align="center">
<a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard"/></a>
<a href="./LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" /></a>
</p>

Header-only C++ libraries containing controllers designed for Lie Groups.

## Dependeces
- [manif](https://github.com/artivis/manif);
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page);
- [cmake](https://cmake.org/);
- [Catch2](https://github.com/catchorg/Catch2) (only for testing).

## Build the library
### Linux / macOs
```sh
git clone https://github.com/GiulioRomualdi/lie-group-controllers.git
cd lie-group-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
If you want to enable tests set the `BUILD_TESTING` option to `ON`.

## Use lie-groups-controllers in your project
**lie-groups-controllers** provides native CMake support which allows the library to be easily used in CMake projects. Please add in your `CMakeLists.txt`
```cmake
project(foo)
find_package(LieGroupsControllers REQUIRED)
add_executable(${PROJECT_NAME} src/foo.cpp)
target_link_libraries(${PROJECT_NAME} LieGroupsControllers::LieGroupsControllers)
```

## Bug reports and support
All types of [issues](https://github.com/GiulioRomualdi/lie-group-controllers/issues/new) are welcome.
