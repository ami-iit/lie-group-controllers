<p align="center">
<h1 align="center">lie-group-controllers </h1>
</p>

<p align="center">
<a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard"/></a>
<a href="./LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" /></a>
</p>

Header-only C++ libraries containing controllers designed for Lie Groups.

## Some theory behind the library

The aim of the library is to contain some controllers designed in lie groups. The library depends only on `Eigen` and [`manif`](https://github.com/artivis/manif).

All the controllers defined in `lie-group-controllers` have in common that they inherit from a templated base class ([CRTP](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern)). It allows one to write generic code abstracting the controller details. This follows the structure of `manif` and `Eigen`.

The library implements two controllers:
1. Proportional Controller (`P controller`)
2. Proportional Derivative Controller (`PD controller`)

The controllers have the following form

|       Proportional Controller      | Proportional Derivative Controller |
|:-----------------------:|:-------------:|
| ![img-f6214bd2482f678b](https://user-images.githubusercontent.com/16744101/89174620-77c5b100-d586-11ea-88f7-318343c13b0f.png)  | ![img-40c85670ed9bec65](https://user-images.githubusercontent.com/16744101/89174628-7b593800-d586-11ea-8219-d3ea2cb70901.png)        |

where `X` and `Xᵈ` are elements of a Lie group. `∘` is the group operator. `ψ` represents an element in the Lie algebra of the Lie group whose coordinates are expressed in `ℝⁿ`.

At the moment, the controllers support all the group defined in `manif`. Namely:
- ℝ(n): Euclidean space with addition.
- SO(2): rotations in the plane.
- SE(2): rigid motion (rotation and translation) in the plane.
- SO(3): rotations in 3D space.
- SE(3): rigid motion (rotation and translation) in 3D space.

Please you can find further information in
```
Modern Robotics: Mechanics, Planning, and Control," Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017, ISBN 9781107156302
```

## Dependeces

- [manif](https://github.com/artivis/manif)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [cmake](https://cmake.org/)
- [Catch2](https://github.com/catchorg/Catch2) (only for testing)

## Build the library

```sh
git clone https://github.com/GiulioRomualdi/lie-group-controllers.git
cd lie-group-controllers
mkdir build && cd build
cmake ../
cmake --build .
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

All types of [issues](https://github.com/dic-iit/lie-group-controllers/issues/new) are welcome.

## Note

The original version of the library can be found [here](https://github.com/GiulioRomualdi/lie-group-controllers).
