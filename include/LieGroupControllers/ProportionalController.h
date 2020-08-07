/**
 * @file ProportionalController.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_PROPORTIONAL_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_PROPORTIONAL_CONTROLLER_H

#include <LieGroupControllers/impl/ProportionalController/Controller.h>

namespace LieGroupControllers
{
    using ProportionalControllerSO3d = ProportionalController<manif::SO3d>;
    using ProportionalControllerSE3d = ProportionalController<manif::SE3d>;
    using ProportionalControllerR3d = ProportionalController<manif::R3d>;
}

#endif
