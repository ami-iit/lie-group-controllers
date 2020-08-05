# Authors: Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

# Set cmake mimimun version
cmake_minimum_required(VERSION 3.5)

project(LieGroupControllers
  VERSION 0.0.1)

# ouptut paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Build shared libs
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Disable C and C++ compiler extensions.
# C/CXX_EXTENSIONS are ON by default to allow the compilers to use extended
# variants of the C/CXX language.
# However, this could expose cross-platform bugs in user code or in the headers
# of third-party dependencies and thus it is strongly suggested to turn
# extensions off.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

# add GNU dirs
include(GNUInstallDirs)

include(CMakePackageConfigHelpers)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
      message(STATUS "Setting build type to 'Release' as none was specified.")
      set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

option(BUILD_TESTING "Create tests using CMake" OFF)
include(CTest)

# Check LieGroupControllers dependencies, find necessary libraries.
include(LieGroupControllersDependencies)

# Set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(LIEGROUPCONTROLLERS_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${LIEGROUPCONTROLLERS_BUILD_TYPES})
endif()

set(LIBRARY_TARGET_NAME LieGroupControllers)

set(${LIBRARY_TARGET_NAME}_PUBLIC_HDR
  include/LieGroupControllers/ProportionalController.h
  )

set(${LIBRARY_TARGET_NAME}_PRIVATE_HDR
  include/LieGroupControllers/impl/traits.h
  include/LieGroupControllers/impl/ControllerBase.h
  )

set(${LIBRARY_TARGET_NAME}_PRIVATE_PROPORTIONAL_HDR
  include/LieGroupControllers/impl/ProportionalController/Controller.h
  include/LieGroupControllers/impl/ProportionalController/ControllerBase.h
  )

set(${LIBRARY_TARGET_NAME}_PRIVATE_PROPORTIONAL_DERIVATIVE_HDR
  include/LieGroupControllers/impl/ProportionalDerivativeController/Controller.h
  include/LieGroupControllers/impl/ProportionalDerivativeController/ControllerBase.h
  )

add_library(${LIBRARY_TARGET_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_include_directories(${LIBRARY_TARGET_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${LIBRARY_TARGET_NAME} INTERFACE Eigen3::Eigen MANIF::manif)

install(TARGETS ${LIBRARY_TARGET_NAME} EXPORT  ${PROJECT_NAME})

install(FILES ${${LIBRARY_TARGET_NAME}_PUBLIC_HDR}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}")

install(FILES ${${LIBRARY_TARGET_NAME}_PRIVATE_HDR}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/impl")

install(FILES ${${LIBRARY_TARGET_NAME}_PRIVATE_PROPORTIONAL_HDR}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/impl/ProportionalController")

install(FILES ${${LIBRARY_TARGET_NAME}_PRIVATE_PROPORTIONAL_DERIVATIVE_HDR}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/impl/ProportionalDerivativeController")

include(InstallBasicPackageFiles)
set(LIEGROUPCONTROLLERS_PUBLIC_DEPENDENCIES manif "Eigen3 3.2.92")

install_basic_package_files(${PROJECT_NAME}
  NAMESPACE LieGroupControllers::
  VERSION ${LieGroupControllers_VERSION}
  COMPATIBILITY AnyNewerVersion
  VARS_PREFIX ${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  DEPENDENCIES ${LIEGROUPCONTROLLERS_PUBLIC_DEPENDENCIES})

## Testing
include(AddLieGroupControllersUnitTest)
add_subdirectory(tests)

include(AddUninstallTarget)
