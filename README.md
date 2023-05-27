<p align="center">
<h1 align="center">lie-group-controllers </h1>
</p>

<p align="center">
<a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard"/></a>
<a href="./LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" /></a>
<a href="https://ami-iit.github.io/lie-group-controllers/doxygen/doc/html/index.html"><img src="https://github.com/ami-iit/lie-group-controllers/workflows/GitHub%20Pages/badge.svg" alt="Size" /></a>
<a href="https://github.com/ami-iit/lie-group-controllers/actions?query=workflow%3A%22C%2B%2B+CI+Workflow%22"><img src="https://github.com/ami-iit/lie-group-controllers/workflows/C++%20CI%20Workflow/badge.svg" alt="Size" /></a>
</p>
<p align="center"> <b>Header-only C++ library containing controllers designed for Lie Groups</b></p>

## Some theory behind the library

The library aims to contain some controllers designed in lie groups. The library depends only on `Eigen` and [`manif`](https://github.com/artivis/manif).

All the controllers defined in `lie-group-controllers` have in common that they inherit from a templated base class ([CRTP](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern)). It allows one to write generic code abstracting the controller details. This follows the structure of `manif` and `Eigen`.

The library implements two controllers:
1. Proportional Controller (`P controller`)
2. Proportional Derivative Controller (`PD controller`)

The controllers have the following form

| Trivialization |                   Proportional Controller                    |              Proportional Derivative Controller              |
| :------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
| Left (default) | $\psi = \psi^d + k_p \text{Log}\left(X^d \circ X^{-1}\right)$ | $\dot{\psi} = \dot{\psi}^d + k_d \left(\psi^d - \psi \right) +  k_p \text{Log}\left(X^d \circ X^{-1}\right)$ |
| Right | $\psi = \psi^d + k_p \text{Log}\left(X^{-1} \circ X^d \right)$ | $\dot{\psi} = \dot{\psi}^d + k_d \left(\psi^d - \psi \right) +  k_p \text{Log}\left(X^{-1}\circ X^d \right)$ |

where `X` and `Xᵈ` are elements of a Lie group. `∘` is the group operator. `ψ` represents an element in the Lie algebra of the Lie group whose coordinates are expressed in `ℝⁿ`.

The controllers support all the groups defined in `manif`. Namely:
- ℝ(n): Euclidean space with addition.
- SO(2): rotations in the plane.
- SE(2): rigid motion (rotation and translation) in the plane.
- SO(3): rotations in 3D space.
- SE(3): rigid motion (rotation and translation) in 3D space.

Please you can find further information in
```
Modern Robotics: Mechanics, Planning, and Control,
Kevin M. Lynch and Frank C. Park,
Cambridge University Press, 2017,
ISBN 9781107156302
```

## Basic Usage
The library implements proportional and proportional derivative controllers on Lie groups. What follows are two simple snippets that you can follow to build and use such controllers. For sake of simplicity, only controllers in SO(3) are shown. The very same applies to the other Lie groups

## Proportional controller SO(3)
```cpp
// set random initial state and zero feedforward
manif::SO3d desiredState, state;
desiredState.setRandom();
state.setRandom();
Eigen::Vector3d feedForward = Eigen::Vector3d::Zero();

// create the controller.
ProportionalControllerSO3d controller;

// In case you want to use the right trivialized controller
// ProportionalControllerTplSO3d<Trivialization::Right> controller;

// set the proportional gain
const double kp = 10;
controller.setGains(kp);

// set the desired state, the feed-forward, and the state
controller.setDesiredState(desiredState);
controller.setFeedForward(feedForward);
controller.setState(state);

// compute the control law
controller.computeControlLaw();
const auto& controlOutput = controller.getControl();
```

## Proportional Derivative controller SO(3)
```cpp
// set random initial state and zero feedforward
manif::SO3d desiredState, state;
desiredState.setRandom();
state.setRandom();
manif::SO3d::Tangent stateDerivative = Eigen::Vector3d::Zero();
manif::SO3d::Tangent desiredStateDerivative = Eigen::Vector3d::Zero();
Eigen::Vector3d feedForward = Eigen::Vector3d::Zero();

// create the controller.
ProportionalDerivativeControllerSO3d controller;

// In case you want to use the right trivialized controller
// ProportionalDerivativeControllerTplSO3d<Trivialization::Right> controller;

// set the proportional and the derivative gains
const double kp = 10;
const double kd = 2 * std::sqrt(kp);
controller.setGains(kp, kd);

// set the desired state, its derivative, the feed-forward, and the state
controller.setDesiredState(desiredState, desiredStateDerivative);
controller.setFeedForward(feedForward);
controller.setState(state, stateDerivative);

// compute the control law
controller.computeControlLaw();
const auto& controlOutput = controller.getControl();
```

## Dependeces

- [manif](https://github.com/artivis/manif)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [cmake](https://cmake.org/)

## Build the library

```console
git clone https://github.com/GiulioRomualdi/lie-group-controllers.git
cd lie-group-controllers
mkdir build && cd build
cmake ../
cmake --build .
[sudo] cmake --build . --target install
```
If you want to enable tests set the `BUILD_TESTING` option to `ON`.

## Use lie-group-controllers in your project

**lie-group-controllers** provides native CMake support which allows the library to be easily used in CMake projects. Please add in your `CMakeLists.txt`

```cmake
project(foo)
find_package(LieGroupControllers REQUIRED)
add_executable(${PROJECT_NAME} src/foo.cpp)
target_link_libraries(${PROJECT_NAME} LieGroupControllers::LieGroupControllers)
```

## Bug reports and support

All types of [issues](https://github.com/ami-iit/lie-group-controllers/issues/new) are welcome.

## Note

The original version of the library can be found [here](https://github.com/GiulioRomualdi/lie-group-controllers).
