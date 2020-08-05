/**
 * @file ProportionalControllerTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// Catch2
#include <catch2/catch.hpp>

#include <manif/manif.h>

#include <LieGroupControllers/ProportionalController.h>
using namespace LieGroupControllers;

TEST_CASE("Proportional Controller [SO(3)]")
{
    manif::SO3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();

    auto feedForward = Eigen::Vector3d::Zero();

    // instantiate the controller
    ProportionalControllerSO3d controller;
    constexpr double kp = 10;
    controller.setGains(kp);
    controller.setDesiredState(desiredState);
    controller.setFeedForward(feedForward);

    // Test the controller
    constexpr double dT = 0.01;
    constexpr std::size_t numberOfIteration = 1e3;
    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();

        // Propagate the dynamics of the system.
        // First of all we get the control output. In this particular case is the angular velocity
        // expressed in the inertial frame.
        // Then the Manifold left plus operator is used
        // state = controlOutputDT  + state should be read as
        // state_k+1 = exp(omega * dT) * state_k
        manif::SO3d::Tangent controlOutputDT = controlOutput.coeffs() * dT;

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

    auto feedForward = Eigen::Matrix<double, 6, 1>::Zero();

    // instantiate the controller
    ProportionalControllerSE3d controller;
    constexpr double kp = 10;
    controller.setGains(kp);
    controller.setDesiredState(desiredState);
    controller.setFeedForward(feedForward);

    // Test the controller
    constexpr double dT = 0.01;
    constexpr std::size_t numberOfIteration = 1e3;
    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();

        // Propagate the dynamics of the system.
        // First of all we get the control output. In this particular case is a 6D-spatial vector
        // expressed in the inertial frame.
        // Then the Manifold left plus operator is used
        // state = controlOutputDT  + state should be read as
        // state_k+1 = exp(v * dT) * state_k
        manif::SE3d::Tangent controlOutputDT = controlOutput.coeffs() * dT;
        state = controlOutputDT + state;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}
