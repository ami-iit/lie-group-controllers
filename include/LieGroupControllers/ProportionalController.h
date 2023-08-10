// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIE_GROUP_CONTROLLERS_PROPORTIONAL_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_PROPORTIONAL_CONTROLLER_H

#include <LieGroupControllers/impl/ProportionalController/Controller.h>

namespace LieGroupControllers
{
template <Trivialization _trivialization>
using ProportionalControllerTplSO3d = ProportionalController<manif::SO3d, _trivialization>;

template <Trivialization _trivialization>
using ProportionalControllerTplSE3d = ProportionalController<manif::SE3d, _trivialization>;

template <Trivialization _trivialization>
using ProportionalControllerTplR3d = ProportionalController<manif::R3d, _trivialization>;

using ProportionalControllerSO3d = ProportionalControllerTplSO3d<Trivialization::Left>;
using ProportionalControllerSE3d = ProportionalControllerTplSE3d<Trivialization::Left>;
using ProportionalControllerR3d = ProportionalControllerTplR3d<Trivialization::Left>;

} // namespace LieGroupControllers

#endif
