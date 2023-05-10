/**
 * @file Controller.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_H

#include <tuple>
#include <type_traits>

#include <Eigen/Dense>

#include <LieGroupControllers/impl/ProportionalController/ControllerBase.h>
#include <manif/manif.h>

namespace LieGroupControllers
{

// Forward declare for type traits specialization
template <typename _LieGroupType, Trivialization _trivialization> class ProportionalController;

namespace internal
{

template <typename _LieGroupType, Trivialization _trivialization>
struct traits<ProportionalController<_LieGroupType, _trivialization>>
{
    using LieGroup = typename manif::LieGroupBase<_LieGroupType>::LieGroup;
    using Tangent = typename manif::LieGroupBase<_LieGroupType>::Tangent;
    using State = std::tuple<LieGroup>;
    using Vector = Tangent;
    using ScalarGains = std::tuple<typename LieGroup::Scalar>;
    using Gains = std::tuple<Eigen::Matrix<typename LieGroup::Scalar, Tangent::DoF, 1>>;
    static constexpr Trivialization trivialization = _trivialization;
};

} // namespace internal
} // namespace LieGroupControllers

namespace LieGroupControllers
{

/**
 * ProporionalDerivativeController describes a proportional controller on Lie Groups.
 * @tparam _LieGroupType describes the Lie Group.
 * @tparam _trivialization is the type of trivialization used by the controller.
 * @note _LieGroupType must derived from manif::LieGroupBase<_LieGroupType>
 */
template <typename _LieGroupType, Trivialization _trivialization>
class ProportionalController
    : public ProportionalControllerBase<ProportionalController<_LieGroupType, _trivialization>>
{
    static_assert(std::is_base_of<manif::LieGroupBase<_LieGroupType>, _LieGroupType>::value,
                  "The template type must derive from 'manif::LieGroupBase<>'");
};

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_CONTROLLER_CONTROLLER_H
