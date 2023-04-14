/**
 * @file ProportionalDerivativeControllerTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// Catch2
#include <catch2/catch.hpp>

#include <manif/manif.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

using namespace LieGroupControllers;

TEST_CASE("Proportional Derivative Controller [SO(3)] with scalar kp and kd")
{
    manif::SO3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();
    manif::SO3d::Tangent stateDerivative = Eigen::Vector3d::Zero();

    const manif::SO3d::Tangent desiredStateDerivative = Eigen::Vector3d::Zero();
    const auto feedForward = Eigen::Vector3d::Zero();

    // Initialize the controller
    ProportionalDerivativeControllerSO3d controller;
    constexpr double kp = 10;
    const double kd = 2 * std::sqrt(kp);
    controller.setGains(kp, kd);
    controller.setDesiredState(desiredState, desiredStateDerivative);
    controller.setFeedForward(feedForward);

    // Test the controller
    constexpr double dT = 0.01;
    constexpr std::size_t numberOfIteration = 1e3;
    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state, stateDerivative);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();

        // Propagate the dynamics of the system.
        // First of all we get the control output. In this particular case is the angular
        // acceleration expressed in the inertial frame. Then the Manifold left plus operator is
        // used state = stateDerivativeDT + state should be read as
        // state_k+1 = exp(omega * dT) * state_k
        manif::SO3d::Tangent stateDerivativeDT = stateDerivative.coeffs() * dT;
        state = stateDerivativeDT + state;

        // here the following operator has been used
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L316-L318
        // and
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L300-L310
        stateDerivative += controlOutput * dT;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}

TEST_CASE("Proportional Derivative Controller [SO(3)] with vector kp and kd")
{
    manif::SO3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();
    manif::SO3d::Tangent stateDerivative = Eigen::Vector3d::Zero();

    const manif::SO3d::Tangent desiredStateDerivative = Eigen::Vector3d::Zero();
    const auto feedForward = Eigen::Vector3d::Zero();

    // Initialize the controller
    ProportionalDerivativeControllerSO3d controller;
    const Eigen::Vector3d kp = Eigen::Vector3d::Constant(10);
    const Eigen::Vector3d kd = 2 * Eigen::Vector3d::Constant(std::sqrt(10));
    controller.setGains(kp, kd);
    controller.setDesiredState(desiredState, desiredStateDerivative);
    controller.setFeedForward(feedForward);

    // Test the controller
    constexpr double dT = 0.01;
    constexpr std::size_t numberOfIteration = 1e3;
    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state, stateDerivative);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();

        // Propagate the dynamics of the system.
        // First of all we get the control output. In this particular case is the angular
        // acceleration expressed in the inertial frame. Then the Manifold left plus operator is
        // used state = stateDerivativeDT + state should be read as
        // state_k+1 = exp(omega * dT) * state_k
        manif::SO3d::Tangent stateDerivativeDT = stateDerivative.coeffs() * dT;
        state = stateDerivativeDT + state;

        // here the following operator has been used
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L316-L318
        // and
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L300-L310
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

    // Initialize the controller
    ProportionalDerivativeControllerSE3d controller;
    constexpr double kp = 10;
    const double kd = 2 * std::sqrt(kp);
    controller.setGains(kp, kd);
    controller.setDesiredState(desiredState, desiredStateDerivative);

    controller.setFeedForward(feedForward);

    // Test the controller
    constexpr double dT = 0.01;
    constexpr std::size_t numberOfIteration = 1e3;
    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state, stateDerivative);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();

        // Propagate the dynamics of the system.
        // First of all we get the control output. In this particular case is the angular
        // acceleration expressed in the inertial frame. Then the Manifold left plus operator is
        // used state = stateDerivativeDT + state should be read as
        // state_k+1 = exp(v * dT) * state_k
        manif::SE3d::Tangent stateDerivativeDT = stateDerivative.coeffs() * dT;
        state = stateDerivativeDT + state;

        // here the following operator has been used
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L316-L318
        // and
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L300-L310
        stateDerivative += controlOutput * dT;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}

TEST_CASE("Proportional Derivative Controller [R3]")
{
    manif::R3d desiredState, state;
    desiredState.setRandom();
    state.setRandom();
    manif::R3d::Tangent stateDerivative = Eigen::Vector3d::Zero();

    const manif::R3d::Tangent desiredStateDerivative = Eigen::Vector3d::Zero();
    const auto feedForward = Eigen::Vector3d::Zero();

    // Initialize the controller
    ProportionalDerivativeControllerR3d controller;
    constexpr double kp = 10;
    const double kd = 2 * std::sqrt(kp);
    controller.setGains(kp, kd);
    controller.setDesiredState(desiredState, desiredStateDerivative);
    controller.setFeedForward(feedForward);

    // Test the controller
    constexpr double dT = 0.01;
    constexpr std::size_t numberOfIteration = 1e3;
    for (std::size_t i = 0; i < numberOfIteration; i++)
    {
        controller.setState(state, stateDerivative);
        controller.computeControlLaw();
        auto controlOutput = controller.getControl();

        // In this specific case the dynamics of the system is a simple double integrator.
        // and the controller is a simple PD controller in R3.
        // Let's check the last statement

        Eigen::Vector3d expectedControlOutput = feedForward
                                                + kd * (desiredStateDerivative.coeffs() - stateDerivative.coeffs())
                                                + kp * (desiredState.coeffs() - state.coeffs());

        REQUIRE(expectedControlOutput.isApprox(controlOutput.coeffs()));

        // Propagate the system dynamics
        manif::R3d::Tangent stateDerivativeDT = stateDerivative.coeffs() * dT;
        state = stateDerivativeDT + state;

        // here the following operator has been used
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L316-L318
        // and
        // https://github.com/artivis/manif/blob/6d07bc65cc5b25c49f1231021be5e61132e5f777/include/manif/impl/tangent_base.h#L300-L310
        stateDerivative += controlOutput * dT;
    }

    // check the error
    auto error = state.compose(desiredState.inverse()).log();
    REQUIRE(error.coeffs().norm() < 1e-4);
}
