# Authors: Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

include(LieGroupControllersFindOptionalDependencies)

#---------------------------------------------
## Required Dependencies
find_package(Eigen3 3.2.92 REQUIRED)
find_package(manif REQUIRED)

#---------------------------------------------
## Optional Dependencies
find_package(Catch2 QUIET)
checkandset_optional_dependency(Catch2)

find_package(VALGRIND QUIET)
checkandset_optional_dependency(VALGRIND)
