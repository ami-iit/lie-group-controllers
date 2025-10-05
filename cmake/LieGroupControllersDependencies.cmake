# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(LieGroupControllersFindOptionalDependencies)

#---------------------------------------------
## Required Dependencies
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)

#---------------------------------------------
## Optional Dependencies
find_package(VALGRIND QUIET)
checkandset_optional_dependency(VALGRIND)
