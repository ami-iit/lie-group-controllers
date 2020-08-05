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
    using Gains = typename internal::traits<_Derived>::Gains; /**< Gains used by the controller */

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
    bool setState(const State& state);

    /**
     * Set the desired state.
     * @param state of the system.
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
};

template <class _Derived>
bool ControllerBase<_Derived>::setState(const ControllerBase<_Derived>::State& state)
{
    return this->derived().setState(state);
}

template <class _Derived>
bool ControllerBase<_Derived>::setDesiredState(const ControllerBase<_Derived>::State& state)
{
    return this->derived().setDesiredState(state);
}

template <class _Derived>
bool ControllerBase<_Derived>::setFeedForward(const ControllerBase<_Derived>::Vector& feedForward)
{
    return this->derived().setFeedForward(feedForward);
}

template <class _Derived> void ControllerBase<_Derived>::setGains(const Gains& gains)
{
    this->derived().setGains(gains);
    return;
}

template <class _Derived> void ControllerBase<_Derived>::computeControlLaw()
{
    this->derived().computeControlLaw();
    return;
}

template <class _Derived>
const typename ControllerBase<_Derived>::Vector& ControllerBase<_Derived>::getControl() const
{
    return this->derived().getControl();
}

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_CONTROLLER_BASE_H
