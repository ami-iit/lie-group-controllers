/**
 * @file traits.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_IMPL_TRAITS_H
#define LIE_GROUP_CONTROLLERS_IMPL_TRAITS_H

namespace LieGroupControllers
{
namespace internal
{

template <typename T> struct traits;

/// @note the following is from the Eigen library
/// here we say once and for all that traits<const T> == traits<T>
///
/// When constness must affect traits, it has to be constness on
/// template parameters on which T itself depends.
/// For example, traits<Map<const T> > != traits<Map<T> >, but
///              traits<const Map<T> > == traits<Map<T> >
template <typename T> struct traits<const T> : traits<T>
{
};

} // namespace internal
} // namespace LieGroupControllers

#endif // LIE_GROUP_CONTROLLERS_IMPL_TRAITS_H
