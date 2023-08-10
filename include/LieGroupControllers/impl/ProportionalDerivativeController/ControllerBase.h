// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
    using ScalarGains = typename ControllerBase<_Derived>::ScalarGains;
    using Gains = typename ControllerBase<_Derived>::Gains;
    using LieGroup = typename ControllerBase<_Derived>::LieGroup;
    static constexpr Trivialization m_trivialization = ControllerBase<_Derived>::trivialization;

private:
    State m_state{LieGroup::Identity(), Vector::Zero()};
    State m_desiredState{LieGroup::Identity(), Vector::Zero()};
    Vector m_feedForward{Vector::Zero()};
    Vector m_controlOutput{Vector::Zero()};
    Gains m_gain{std::tuple_element<0, Gains>::type::Zero(),
                 std::tuple_element<0, Gains>::type::Zero()};

public:
    /**
     * Set the control state.
     * @param state of the system.
     * @note for the ProportionalDerivativeController the state is given by the element of the Lie
     * group and one of it's tangent space at the identity.
     * @return true in case of success, false otherwise.
     */
    bool setStateImpl(const State& state);

    /**
     * Set the desired state.
     * @param state of the system.
     * @note for the ProportionalDerivativeController the state is given by the element of the Lie
     * group and one of it's tangent space at the identity.
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
bool ProportionalDerivativeControllerBase<_Derived>::setStateImpl(const State& state)
{
    m_state = state;
    return true;
}

template <typename _Derived>
bool ProportionalDerivativeControllerBase<_Derived>::setDesiredStateImpl(const State& state)
{
    m_desiredState = state;
    return true;
}

template <typename _Derived>
bool ProportionalDerivativeControllerBase<_Derived>::setFeedForwardImpl(const Vector& feedForward)
{
    m_feedForward = feedForward;
    return true;
}

template <typename _Derived>
void ProportionalDerivativeControllerBase<_Derived>::setGainsImpl(const Gains& gain)
{
    m_gain = gain;
}

template <typename _Derived>
void ProportionalDerivativeControllerBase<_Derived>::setGainsImpl(const ScalarGains& gain)
{
    std::get<0>(m_gain).setConstant(std::get<0>(gain));
    std::get<1>(m_gain).setConstant(std::get<1>(gain));
}

template <typename _Derived>
void ProportionalDerivativeControllerBase<_Derived>::computeControlLawImpl()
{
    const typename std::tuple_element<0, Gains>::type& kp = std::get<0>(m_gain);
    const typename std::tuple_element<1, Gains>::type& kd = std::get<1>(m_gain);

    const auto& state = std::get<0>(m_state);
    const auto& stateDerivative = std::get<1>(m_state);

    const auto& desiredState = std::get<0>(m_desiredState);
    const auto& desiredStateDerivative = std::get<1>(m_desiredState);

    // evaluate error state
    Vector errorState;
    if constexpr (m_trivialization == Trivialization::Left)
    {
        // please read it as
        // log(X_d * X ^-1)^\vee
        // Indeed here log() is a sequence of an actual logarithm mapping of the group plus a vee
        // operator.
        errorState = desiredState.compose(state.inverse()).log();
    } else
    {
        static_assert(m_trivialization == Trivialization::Right, "Expecting right trivialization");
        // please read it as
        // log(X ^-1 * X_d)^\vee
        // Indeed here log() is a sequence of an actual logarithm mapping of the group plus a vee
        // operator.
        errorState = (state.inverse().compose(desiredState)).log();
    }
    Vector errorStateDerivative = desiredStateDerivative - stateDerivative;

    // compute the control law
    m_controlOutput = m_feedForward;
    m_controlOutput += kd.asDiagonal() * errorStateDerivative.coeffs();
    m_controlOutput += kp.asDiagonal() * errorState.coeffs();
}

template <typename _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::Vector&
ProportionalDerivativeControllerBase<_Derived>::getControlImpl() const
{
    return m_controlOutput;
}

template <class _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::State&
ProportionalDerivativeControllerBase<_Derived>::getStateImpl() const
{
    return m_state;
}

template <class _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::State&
ProportionalDerivativeControllerBase<_Derived>::getDesiredStateImpl() const
{
    return m_desiredState;
}

template <class _Derived>
const typename ProportionalDerivativeControllerBase<_Derived>::Vector&
ProportionalDerivativeControllerBase<_Derived>::getFeedForwardImpl() const
{
    return m_feedForward;
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_BASE_H
