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
    using State = typename ControllerBase<_Derived>::State;
    using Vector = typename ControllerBase<_Derived>::Vector;
    using Gains = typename ControllerBase<_Derived>::Gains;
    using LieGroup = typename ControllerBase<_Derived>::LieGroup;

    State m_state{LieGroup::Identity(), Vector::Zero()};
    State m_desiredState{LieGroup::Identity(), Vector::Zero()};
    Vector m_feedForward{Vector::Zero()};
    Vector m_controlOutput{Vector::Zero()};
    Gains m_gain{0, 0};

public:
    /**
     * Set the control state.
     * @param state of the system.
     * @note for the ProportionalDerivativeController the state is given by the element of the Lie
     * group and one of it's tangent space at the identity.
     * @return true in case of success, false otherwise.
     */
    bool setState(const State& state);

    /**
     * Set the desired state.
     * @param state of the system.
     * @note for the ProportionalDerivativeController the state is given by the element of the Lie
     * group and one of it's tangent space at the identity.
     * @return true in case of success, false otherwise.
     */
    bool setDesiredState(const State& state);

    /**
     * Set the feedforward term of the controller.
     * @param feedforward is a vector of the tangent space of the the group.
     * @return true in case of success, false otherwise.
     */
    bool setFeedForward(const Vector& feedForward);

    /**
     * Set the controller gains.
     * @param gains contains the controller gains.
     * @note for the ProportionalController the gain is simply a double.
     */
    void setGains(const Gains& gains);

    /**
     * Evaluate the control law.
     */
    void computeControlLaw();

    /**
     * Get the control signal.
     * @return a vector containing the control effort.
     * @note Please call this function only after computeControlLaw().
     * @note In the current implementation the control effort belongs to the tangent space at the
     * identity, $\fT_\eps \mathcal{M}$\f. Please use the Adjoint transformation to convert the
     * express the vector in a different tangent space.
     */
    const Vector& getControl() const;

    /**
     * Get the state of the system.
     * @return the state of the system.
     */
    const State& getState() const;

    /**
     * Get the desired state of the system.
     * @return the state of the system.
     */
    const State& getDesiredState() const;

    /**
     * Get the feedforward term.
     * @return the controller feedforward.
     */
    const Vector& getFeedForward() const;
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
void ProportionalDerivativeControllerBase<_Derived>::setGains(const Gains& gain)
{
    m_gain = gain;
}

template <typename _Derived>
void ProportionalDerivativeControllerBase<_Derived>::computeControlLaw()
{
    const double& kp = std::get<0>(m_gain);
    const double& kd = std::get<1>(m_gain);

    const auto& state = std::get<0>(m_state);
    const auto& stateDerivative = std::get<1>(m_state);

    const auto& desiredState = std::get<0>(m_desiredState);
    const auto& desiredStateDerivative = std::get<1>(m_desiredState);

    // evaluate state
    Vector errorState = (desiredState.compose(state.inverse())).log();
    Vector errorStateDerivative = desiredStateDerivative - stateDerivative;

    // compute the control law
    m_controlOutput = m_feedForward + errorStateDerivative * kd + errorState * kp;
}

template <typename _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::Vector&
ProportionalDerivativeControllerBase<_Derived>::getControl() const
{
    return m_controlOutput;
}

template <class _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::State&
ProportionalDerivativeControllerBase<_Derived>::getState() const
{
    return m_state;
}

template <class _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::State&
ProportionalDerivativeControllerBase<_Derived>::getDesiredState() const
{
    return m_desiredState;
}

template <class _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::Vector&
ProportionalDerivativeControllerBase<_Derived>::getFeedForward() const
{
    return m_feedForward;
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_BASE_H
