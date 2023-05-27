/**
 * @file ControllerBase.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_CONTROLLER_BASE_H
#define LIE_GROUP_CONTROLLERS_IMPL_CONTROLLER_BASE_H

#include <LieGroupControllers/impl/traits.h>

namespace LieGroupControllers
{

enum class Trivialization
{
    Left,
    Right
};

/**
 * ControllerBase describes a base controller
 * @tparam _Derived type of the Derived class. Necessary to implement the Curiously recurring
 * template pattern.
 */
template <class _Derived> class ControllerBase
{
public:
    using State = typename internal::traits<_Derived>::State; /**< State type */
    using Vector = typename internal::traits<_Derived>::Vector; /**< Here we consider a Vector an
                                                                   element of the lie algebra of
                                                                   the Group. */
    using ScalarGains = typename internal::traits<_Derived>::ScalarGains; /** TODO */
    using Gains = typename internal::traits<_Derived>::Gains; /**< Gains used by the controller */
    using LieGroup = typename internal::traits<_Derived>::LieGroup; /**< Lie Group */

    static constexpr Trivialization trivialization = internal::traits<_Derived>::trivialization;

private:
    _Derived& derived()
    {
        return *static_cast<_Derived*>(this);
    }
    const _Derived& derived() const
    {
        return *static_cast<const _Derived*>(this);
    }

public:
    /**
     * Set the control state.
     * @param state of the system.
     * @return true in case of success, false otherwise.
     */
    template <typename... Ts> bool setState(Ts&&... state);

    /**
     * Set the desired state.
     * @param state of the system.
     * @return true in case of success, false otherwise.
     */
    template <typename... Ts> bool setDesiredState(Ts&&... state);

    /**
     * Set the feedforward term of the controller.
     * @param feedforward is a vector of the tangent space of the the group.
     * @return true in case of success, false otherwise.
     */
    bool setFeedForward(const Vector& feedForward);

    /**
     * Set the controller gains.
     * @param gains contains the controller gain.
     */
    template <typename... Ts> void setGains(Ts&&... gains);

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

template <class _Derived>
template <typename... Ts>
bool ControllerBase<_Derived>::setState(Ts&&... state)
{
    return this->derived().setStateImpl(std::make_tuple(std::forward<Ts>(state)...));
}

template <class _Derived>
template <typename... Ts>
bool ControllerBase<_Derived>::setDesiredState(Ts&&... state)
{
    return this->derived().setDesiredStateImpl(std::make_tuple(std::forward<Ts>(state)...));
}

template <class _Derived>
bool ControllerBase<_Derived>::setFeedForward(const ControllerBase<_Derived>::Vector& feedForward)
{
    return this->derived().setFeedForwardImpl(feedForward);
}

template <class _Derived>
template <typename... Ts>
void ControllerBase<_Derived>::setGains(Ts&&... gains)
{
    this->derived().setGainsImpl(std::make_tuple(std::forward<Ts>(gains)...));
}

template <class _Derived> void ControllerBase<_Derived>::computeControlLaw()
{
    this->derived().computeControlLawImpl();
    return;
}

template <class _Derived>
const typename ControllerBase<_Derived>::Vector& ControllerBase<_Derived>::getControl() const
{
    return this->derived().getControlImpl();
}

template <class _Derived>
const typename ControllerBase<_Derived>::State& ControllerBase<_Derived>::getState() const
{
    return this->derived().getStateImpl();
}

template <class _Derived>
const typename ControllerBase<_Derived>::State& ControllerBase<_Derived>::getDesiredState() const
{
    return this->derived().getDesiredStateImpl();
}

template <class _Derived>
const typename ControllerBase<_Derived>::Vector& ControllerBase<_Derived>::getFeedForward() const
{
    return this->derived().getFeedForwardImpl();
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_CONTROLLER_BASE_H
