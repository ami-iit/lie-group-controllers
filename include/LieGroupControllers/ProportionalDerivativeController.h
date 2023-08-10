// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIE_GROUP_CONTROLLERS_PROPORTIONAL_DERIVATIVE_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_PROPORTIONAL_DERIVATIVE_CONTROLLER_H

#include <LieGroupControllers/impl/ProportionalDerivativeController/Controller.h>

namespace LieGroupControllers
{

template <Trivialization _trivialization>
using ProportionalDerivativeControllerTplSO3d
    = ProportionalDerivativeController<manif::SO3d, _trivialization>;

template <Trivialization _trivialization>
using ProportionalDerivativeControllerTplSE3d
    = ProportionalDerivativeController<manif::SE3d, _trivialization>;

template <Trivialization _trivialization>
using ProportionalDerivativeControllerTplR3d
    = ProportionalDerivativeController<manif::R3d, _trivialization>;

using ProportionalDerivativeControllerSO3d
    = ProportionalDerivativeControllerTplSO3d<Trivialization::Left>;
using ProportionalDerivativeControllerSE3d
    = ProportionalDerivativeControllerTplSE3d<Trivialization::Left>;
using ProportionalDerivativeControllerR3d
    = ProportionalDerivativeControllerTplR3d<Trivialization::Left>;

} // namespace LieGroupControllers

#endif
