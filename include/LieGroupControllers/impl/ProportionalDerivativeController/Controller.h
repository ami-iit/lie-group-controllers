// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_H

#include <type_traits>

#include <manif/manif.h>

#include <LieGroupControllers/impl/ProportionalDerivativeController/ControllerBase.h>

namespace LieGroupControllers
{

// Forward declare for type traits specialization
template <typename _LieGroupType, Trivialization _trivialization>
class ProportionalDerivativeController;

namespace internal
{

template <typename _LieGroupType, Trivialization _trivialization>
struct traits<ProportionalDerivativeController<_LieGroupType, _trivialization>>
{
    using LieGroup = typename manif::LieGroupBase<_LieGroupType>::LieGroup;
    using Tangent = typename manif::LieGroupBase<_LieGroupType>::Tangent;
    using State = std::tuple<LieGroup, Tangent>;
    using Vector = Tangent;
    using ScalarGains = std::tuple<typename LieGroup::Scalar, typename LieGroup::Scalar>;
    using Gains = std::tuple<Eigen::Matrix<typename LieGroup::Scalar, Tangent::DoF, 1>,
                             Eigen::Matrix<typename LieGroup::Scalar, Tangent::DoF, 1>>;
    static constexpr Trivialization trivialization = _trivialization;
};

} // namespace internal
} // namespace LieGroupControllers

namespace LieGroupControllers
{

/**
 * ProporionalDerivativeController describes a PD controller on Lie Groups.
 * @tparam _LieGroupType describes the Lie Group.
 * @tparam _trivialization is the type of trivialization used by the controller.
 * @note _LieGroupType must derived from manif::LieGroupBase<_LieGroupType>
 */
template <typename _LieGroupType, Trivialization _trivialization>
class ProportionalDerivativeController
    : public ProportionalDerivativeControllerBase<
          ProportionalDerivativeController<_LieGroupType, _trivialization>>
{
    static_assert(std::is_base_of<manif::LieGroupBase<_LieGroupType>, _LieGroupType>::value,
                  "The template type must derive from 'manif::LieGroupBase<>'");
};

} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_PROPORTIONAL_DERIVATIVE_CONTROLLER_CONTROLLER_H
