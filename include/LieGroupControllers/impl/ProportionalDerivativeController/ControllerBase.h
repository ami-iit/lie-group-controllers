/**
 * @file ControllerBase.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_BASE_H
#define LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_BASE_H

#include <LieGroupControllers/impl/ControllerBase.h>
#include <tuple>

namespace LieGroupControllers
{
template <typename _Derived>
class ProportionalDerivativeControllerBase : public ControllerBase<_Derived>
{

public:
    using State = typename ControllerBase<_Derived>::State;
    using Vector = typename ControllerBase<_Derived>::Vector;
    using Gains = typename ControllerBase<_Derived>::Gains;

private:
    State m_state;
    State m_desiredState;
    Vector m_feedForward;
    Vector m_controlOutput;
    Gains m_gain;

public:
    bool setState(const State& state);

    bool setDesiredState(const State& state);

    bool setFeedForward(const Vector& feedForward);

    void computeControlLaw();

    void setGain(const Gains& gain);

    const Vector& getControl() const;
};

template <typename _Derived>
bool ProportionalDerivativeControllerBase<_Derived>::setState(const State& state)
{
    m_state = state;
    return true;
}

template <typename _Derived>
bool ProportionalDerivativeControllerBase<_Derived>::setDesiredState(const State& state)
{
    m_desiredState = state;
    return true;
}

template <typename _Derived>
bool ProportionalDerivativeControllerBase<_Derived>::setFeedForward(const Vector& feedForward)
{
    m_feedForward = feedForward;
    return true;
}

template <typename _Derived>
void ProportionalDerivativeControllerBase<_Derived>::setGain(const Gains& gain)
{
    m_gain = gain;
}

template <typename _Derived>
void ProportionalDerivativeControllerBase<_Derived>::computeControlLaw()
{
    const auto& kp = std::get<0>(m_gain);
    const auto& kd = std::get<1>(m_gain);

    const auto& state = std::get<0>(m_state);
    const auto& stateDerivative = std::get<1>(m_state);

    const auto& desiredState = std::get<0>(m_desiredState);
    const auto& desiredStateDerivative = std::get<1>(m_desiredState);

    // evaluate state
    auto errorState = (desiredState.compose(state.inverse())).log();
    auto errorStateDerivative = desiredStateDerivative - stateDerivative;

    m_controlOutput
        = m_feedForward.coeffs() + kd * errorStateDerivative.coeffs() + kp * errorState.coeffs();
}

template <typename _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::Vector&
ProportionalDerivativeControllerBase<_Derived>::getControl() const
{
    return m_controlOutput;
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_BASE_H
