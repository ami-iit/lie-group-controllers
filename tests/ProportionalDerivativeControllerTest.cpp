/**
 * @file ProportionalDerivativeControllerTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// Catch2
#include <catch2/catch.hpp>

#include <LieGroupControllers/ProportionalDerivativeController.h>
using namespace LieGroupControllers;

TEST_CASE("Proportional Derivative Controller [SO(3)]")
{
    manif::SO3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();
    manif::SO3d::Tangent stateDerivative = Eigen::Vector3d::Zero();
    const manif::SO3d::Tangent desiredStateDerivative = Eigen::Vector3d::Zero();
    const auto feedForward = Eigen::Vector3d::Zero();

    constexpr double dT = 0.01;
    constexpr double kp = 10;
    constexpr double kd = 2 * std::sqrt(kp);
    constexpr std::size_t numberOfIteration = 1e3;

    ProportionalDerivativeControllerSO3 controller;
    controller.setGain({kp, kd});
    controller.setDesiredState({desiredState, desiredStateDerivative});

    controller.setFeedForward(feedForward);

    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState({state, stateDerivative});
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();
        manif::SO3d::Tangent stateDerivativeDT = stateDerivative.coeffs() * dT;
        state = stateDerivativeDT + state;

        manif::SO3d::Tangent controlOutputDT = controlOutput.coeffs() * dT;
        stateDerivative += controlOutput * dT;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}

TEST_CASE("Proportional Derivative Controller [SE(3)]")
{
    manif::SE3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();
    manif::SE3d::Tangent stateDerivative = Eigen::Matrix<double, 6, 1>::Zero();
    const manif::SE3d::Tangent desiredStateDerivative = Eigen::Matrix<double, 6, 1>::Zero();
    const auto feedForward = Eigen::Matrix<double, 6, 1>::Zero();

    constexpr double dT = 0.01;
    constexpr double kp = 10;
    constexpr double kd = 2 * std::sqrt(kp);
    constexpr std::size_t numberOfIteration = 1e3;

    ProportionalDerivativeControllerSE3 controller;
    controller.setGain({kp, kd});
    controller.setDesiredState({desiredState, desiredStateDerivative});

    controller.setFeedForward(feedForward);

    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState({state, stateDerivative});
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();
        manif::SE3d::Tangent stateDerivativeDT = stateDerivative.coeffs() * dT;
        state = stateDerivativeDT + state;

        manif::SE3d::Tangent controlOutputDT = controlOutput.coeffs() * dT;
        stateDerivative += controlOutput * dT;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}
