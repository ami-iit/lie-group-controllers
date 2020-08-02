/**
 * @file Controller.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_H

#include <type_traits>

#include <LieGroupControllers/impl/ProportionalDerivativeController/ControllerBase.h>
#include <manif/manif.h>

namespace LieGroupControllers
{

// Forward declare for type traits specialization
template <typename _LieGroupType> class ProportionalDerivativeController;

namespace internal
{

template <typename _LieGroupType> struct traits<ProportionalDerivativeController<_LieGroupType>>
{
    using LieGroup = typename manif::LieGroupBase<_LieGroupType>::LieGroup;
    using Tangent = typename manif::LieGroupBase<_LieGroupType>::Tangent;
    using State = std::tuple<LieGroup, Tangent>;
    using Vector = Tangent;
    using Gains = std::tuple<double, double>;
};

} // namespace internal
} // namespace LieGroupControllers

namespace LieGroupControllers
{

template <typename _LieGroupType>
class ProportionalDerivativeController
    : public ProportionalDerivativeControllerBase<ProportionalDerivativeController<_LieGroupType>>
{
    static_assert(std::is_base_of<manif::LieGroupBase<_LieGroupType>, _LieGroupType>::value,
                  "The type must derive from manif");
};

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_H
