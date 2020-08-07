/**
 * @file ProportionalDerivativeController.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_PROPORTIONAL_DERIVATIVE_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_PROPORTIONAL_DERIVATIVE_CONTROLLER_H

#include <LieGroupControllers/impl/ProportionalDerivativeController/Controller.h>

namespace LieGroupControllers
{
    using ProportionalDerivativeControllerSO3d = ProportionalDerivativeController<manif::SO3d>;
    using ProportionalDerivativeControllerSE3d = ProportionalDerivativeController<manif::SE3d>;
    using ProportionalDerivativeControllerR3d = ProportionalDerivativeController<manif::R3d>;
}

#endif
