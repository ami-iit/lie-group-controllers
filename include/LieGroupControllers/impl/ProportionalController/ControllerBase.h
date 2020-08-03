/**
 * @file ControllerBase.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H
#define LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H

#include <LieGroupControllers/impl/ControllerBase.h>

namespace LieGroupControllers
{

template <typename _Derived> class ProportionalControllerBase : public ControllerBase<_Derived>
{
    using State = typename ControllerBase<_Derived>::State;
    using Vector = typename ControllerBase<_Derived>::Vector;
    using Gains = typename ControllerBase<_Derived>::Gains;

    State m_state;
    State m_desiredState;
    Vector m_feedForward;
    Vector m_controlOutput;
    Gains m_gain;

public:
    /**
     * Set the control state.
     * @param state of the system.
     * @note for the ProportionalController the state is given by the element of the Lie group.
     * @return true in case of success, false otherwise.
     */
    bool setState(const State& state);

    /**
     * Set the desired state.
     * @param state of the system.
     * @note for the ProportionalController the state is given by the element of the Lie group.
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
     * @note In the current implementation the control effort belongs to the langet space at the
     * identity, $\fT_\eps \mathcal{M}$\f. Please use the Adjoint transformation to convert the
     * express the vector in a different tangent space.
     */
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

template <typename _Derived> void ProportionalControllerBase<_Derived>::setGains(const Gains& gain)
{
    m_gain = gain;
}

template <typename _Derived> void ProportionalControllerBase<_Derived>::computeControlLaw()
{
    // please read it as
    // log(X_d * X ^-1)^\vee
    // Indeed here log() is a sequence of an actual logarithm mapping of the group plus a vee
    // operator.
    auto error = (m_desiredState.compose(m_state.inverse())).log();
    m_controlOutput = m_feedForward + error * m_gain;
}

template <typename _Derived>
const typename ProportionalControllerBase<_Derived>::Vector&
ProportionalControllerBase<_Derived>::getControl() const
{
    return m_controlOutput;
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H
