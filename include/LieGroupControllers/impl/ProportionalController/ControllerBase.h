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

template <typename _Derived>
class ProportionalControllerBase : public ControllerBase<_Derived>
{
    using State = typename ControllerBase<_Derived>::State;
    using Vector = typename ControllerBase<_Derived>::Vector;
    using ScalarGains = typename ControllerBase<_Derived>::ScalarGains;
    using Gains = typename ControllerBase<_Derived>::Gains;

    State m_state{std::tuple_element<0, State>::type::Identity()};
    State m_desiredState{std::tuple_element<0, State>::type::Identity()};
    Vector m_feedForward{Vector::Zero()};
    Vector m_controlOutput{Vector::Zero()};
    Gains m_gain{std::tuple_element<0, Gains>::type::Zero()};

public:
    /**
     * Set the control state.
     * @param state of the system.
     * @note for the ProportionalController the state is given by the element of the Lie group.
     * @return true in case of success, false otherwise.
     */
    bool setStateImpl(const State& state);

    /**
     * Set the desired state.
     * @param state of the system.
     * @note for the ProportionalController the state is given by the element of the Lie group.
     * @return true in case of success, false otherwise.
     */
    bool setDesiredStateImpl(const State& state);

    /**
     * Set the feedforward term of the controller.
     * @param feedforward is a vector of the tangent space of the the group.
     * @return true in case of success, false otherwise.
     */
    bool setFeedForwardImpl(const Vector& feedForward);

    /**
     * Set the controller gains.
     * @param gains contains the controller gains.
     */
    void setGainsImpl(const Gains& gains);

    /**
     * Set the controller gains.
     * @param gains contains the controller gains.
     */
    void setGainsImpl(const ScalarGains& gains);

    /**
     * Evaluate the control law.
     */
    void computeControlLawImpl();

    /**
     * Get the control signal.
     * @return a vector containing the control effort.
     * @note Please call this function only after computeControlLaw().
     * @note In the current implementation the control effort belongs to the tangent space at the
     * identity, $\fT_\eps \mathcal{M}$\f. Please use the Adjoint transformation to convert the
     * express the vector in a different tangent space.
     */
    const Vector& getControlImpl() const;

    /**
     * Get the state of the system.
     * @return the state of the system.
     */
    const State& getStateImpl() const;

    /**
     * Get the desired state of the system.
     * @return the state of the system.
     */
    const State& getDesiredStateImpl() const;

    /**
     * Get the feedforward term.
     * @return the controller feedforward.
     */
    const Vector& getFeedForwardImpl() const;
};

template <typename _Derived>
bool ProportionalControllerBase<_Derived>::setStateImpl(const State& state)
{
    m_state = state;
    return true;
}

template <typename _Derived>
bool ProportionalControllerBase<_Derived>::setDesiredStateImpl(const State& state)
{
    m_desiredState = state;
    return true;
}

template <typename _Derived>
bool ProportionalControllerBase<_Derived>::setFeedForwardImpl(const Vector& feedForward)
{
    m_feedForward = feedForward;
    return true;
}

template <typename _Derived> void ProportionalControllerBase<_Derived>::setGainsImpl(const Gains& gain)
{
    m_gain = gain;
}

template <typename _Derived>
void ProportionalControllerBase<_Derived>::setGainsImpl(const ScalarGains& gain)
{
    std::get<0>(m_gain).setConstant(std::get<0>(gain));
}

template <typename _Derived> void ProportionalControllerBase<_Derived>::computeControlLawImpl()
{
    // please read it as
    // log(X_d * X ^-1)^\vee
    // Indeed here log() is a sequence of an actual logarithm mapping of the group plus a vee
    // operator.
    auto error = (std::get<0>(m_desiredState).compose(std::get<0>(m_state).inverse())).log();
    m_controlOutput = m_feedForward + std::get<0>(m_gain).asDiagonal() * error.coeffs();
}

template <typename _Derived>
const typename ProportionalControllerBase<_Derived>::Vector&
ProportionalControllerBase<_Derived>::getControlImpl() const
{
    return m_controlOutput;
}

template <class _Derived>
const typename ProportionalControllerBase<_Derived>::State&
ProportionalControllerBase<_Derived>::getStateImpl() const
{
    return m_state;
}

template <class _Derived>
const typename ProportionalControllerBase<_Derived>::State&
ProportionalControllerBase<_Derived>::getDesiredStateImpl() const
{
    return m_desiredState;
}

template <class _Derived>
const typename ProportionalControllerBase<_Derived>::Vector&
ProportionalControllerBase<_Derived>::getFeedForwardImpl() const
{
    return m_feedForward;
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_BASE_H
