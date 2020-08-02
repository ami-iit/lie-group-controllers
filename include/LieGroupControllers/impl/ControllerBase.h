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

template <class _Derived> class ControllerBase
{
public:

    using State = typename internal::traits<_Derived>::State;
    using Vector = typename internal::traits<_Derived>::Vector;
    using Gains = typename internal::traits<_Derived>::Gains;

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
    bool setState(const State& state);

    bool setDesiredState(const State& state);

    bool setFeedForward(const Vector& feedForward);

    void setGains(const Gains& gains);

    void computeControlLaw();

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
