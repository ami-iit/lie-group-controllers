/**
 * @file ControllerBase.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H
#define LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H

#include <LieGroupControllers/impl/ControllerBase.h>
#include <tuple>

namespace LieGroupControllers
{
template <typename _Derived>
class ProportionalControllerBase : public ControllerBase<_Derived>
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

template <typename _Derived> bool ProportionalControllerBase<_Derived>::setState(const State& state)
{
    m_state = state;
    return true;
}

template <typename _Derived>
bool ProportionalControllerBase<_Derived>::setDesiredState(const State& state)
{
    m_desiredState = state;
    return true;
}

template <typename _Derived>
bool ProportionalControllerBase<_Derived>::setFeedForward(const Vector& feedForward)
{
    m_feedForward = feedForward;
    return true;
}

template <typename _Derived>
void ProportionalControllerBase<_Derived>::setGain(const Gains& gain)
{
    m_gain = gain;
}

template <typename _Derived> void ProportionalControllerBase<_Derived>::computeControlLaw()
{
    auto error = (m_desiredState.compose(m_state.inverse())).log();
    m_controlOutput = m_feedForward.coeffs() + m_gain * error.coeffs();
}

template <typename _Derived>
const typename ProportionalControllerBase<_Derived>::Vector&
ProportionalControllerBase<_Derived>::getControl() const
{
    return m_controlOutput;
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H
