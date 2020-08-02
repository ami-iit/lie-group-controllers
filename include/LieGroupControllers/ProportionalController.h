/**
 * @file ProportinalController.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and  distributed under the terms of
 * the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef LIE_GROUP_CONTROLLERS_PROPORTIONAL_CONTROLLER_H
#define LIE_GROUP_CONTROLLERS_PROPORTIONAL_CONTROLLER_H

#include <LieGroupControllers/impl/ProportionalController/ProportionalController.h>

namespace LieGroupControllers
{
    using ProportinalControllerSO3 = ProportionalController<manif::SO3d>;
    using ProportinalControllerSE3 = ProportionalController<manif::SE3d>;
}

#endif
