/**
 * @file ProportionalControllerTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// Catch2
#include <catch2/catch.hpp>

#include <LieGroupControllers/ProportionalController.h>
using namespace LieGroupControllers;

TEST_CASE("Proportional Controller [SO(3)]")
{
    manif::SO3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();

    constexpr double dT = 0.01;
    constexpr double kp = 10;
    constexpr std::size_t numberOfIteration = 1e3;

    ProportinalControllerSO3 controller;
    controller.setGain(kp);
    controller.setDesiredState(desiredState);

    auto feedForward = Eigen::Vector3d::Zero();
    controller.setFeedForward(feedForward);

    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();
        decltype(controlOutput) controlOutputDT = controlOutput.coeffs() * dT;
        state = controlOutputDT + state;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}

TEST_CASE("Proportional Controller [SE(3)]")
{
    manif::SE3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();

    constexpr double dT = 0.01;
    constexpr double kp = 10;
    constexpr std::size_t numberOfIteration = 1e3;

    ProportinalControllerSE3 controller;
    controller.setGain(kp);
    controller.setDesiredState(desiredState);

    auto feedForward = Eigen::Matrix<double, 6, 1>::Zero();
    controller.setFeedForward(feedForward);

    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();
        decltype(controlOutput) controlOutputDT = controlOutput.coeffs() * dT;

        // propagate the dynamics
        state = controlOutputDT + state;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}
