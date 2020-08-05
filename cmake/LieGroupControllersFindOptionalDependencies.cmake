# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This module checks if all the dependencies are installed and if the
# dependencies to build some parts are satisfied.
# For every dependency, it creates the following variables:
#
# LIEGROUPCONTROLLERS_USE_${Package}: Can be disabled by the user if he doesn't want to use that
#                      dependency.
# LIEGROUPCONTROLLERS_HAS_${Package}: Internal flag. It should be used to check if a part of
#                      LIEGROUPCONTROLLERS should be built. It is on if LIEGROUPCONTROLLERS_USE_${Package}
#                      is on and either the package was found or will be built.
# LIEGROUPCONTROLLERS_BUILD_${Package}: Internal flag. Used to check if LIEGROUPCONTROLLERS has to build an
#                        external package.
# LIEGROUPCONTROLLERS_BUILD_DEPS_${Package}: Internal flag. Used to check if dependencies
#                             required to build the package are available.
# LIEGROUPCONTROLLERS_HAS_SYSTEM_${Package}: Internal flag. Used to check if the package is
#                             available on the system.
# LIEGROUPCONTROLLERS_USE_SYSTEM_${Package}: This flag is shown only for packages in the
#                             extern folder that were also found on the system
#                             (TRUE by default). If this flag is enabled, the
#                             system installed library will be used instead of
#                             the version shipped within the framework.


include(CMakeDependentOption)

# Check if a package is installed and set some cmake variables
macro(checkandset_optional_dependency package)

  set(PREFIX "LIEGROUPCONTROLLERS")

  string(TOUPPER ${package} PKG)

  # LIEGROUPCONTROLLERS_HAS_SYSTEM_${package}
  if(${package}_FOUND OR ${PKG}_FOUND)
    set(${PREFIX}_HAS_SYSTEM_${package} TRUE)
  else()
    set(${PREFIX}_HAS_SYSTEM_${package} FALSE)
  endif()

  # LIEGROUPCONTROLLERS_USE_${package}
  cmake_dependent_option(${PREFIX}_USE_${package} "Use package ${package}" TRUE
                         ${PREFIX}_HAS_SYSTEM_${package} FALSE)
  mark_as_advanced(${PREFIX}_USE_${package})

  # LIEGROUPCONTROLLERS_USE_SYSTEM_${package}
  set(${PREFIX}_USE_SYSTEM_${package} ${${PREFIX}_USE_${package}} CACHE INTERNAL "Use system-installed ${package}, rather than a private copy (recommended)" FORCE)
  if(NOT "${package}" STREQUAL "${PKG}")
    unset(${PREFIX}_USE_SYSTEM_${PKG} CACHE)
  endif()

  # LIEGROUPCONTROLLERS_HAS_${package}
  if(${${PREFIX}_HAS_SYSTEM_${package}})
    set(${PREFIX}_HAS_${package} ${${PREFIX}_USE_${package}})
  else()
    set(${PREFIX}_HAS_${package} FALSE)
  endif()

endmacro()

macro(LIEGROUPCONTROLLERS_DEPENDENT_OPTION _option _doc _default _deps _force)

  if(DEFINED ${_option})
    get_property(_option_strings_set CACHE ${_option} PROPERTY STRINGS SET)
    if(_option_strings_set)
      # If the user thinks he is smarter than the machine, he deserves an error
      get_property(_option_strings CACHE ${_option} PROPERTY STRINGS)
      list(GET _option_strings 0 _option_strings_first)
      string(REGEX REPLACE ".+\"(.+)\".+" "\\1" _option_strings_first "${_option_strings_first}")
      list(LENGTH _option_strings _option_strings_length)
      math(EXPR _option_strings_last_index "${_option_strings_length} - 1")
      list(GET _option_strings ${_option_strings_last_index} _option_strings_last)
      if("${${_option}}" STREQUAL "${_option_strings_last}")
        message(SEND_ERROR "That was a trick, you cannot outsmart me! I will never let you win! ${_option} stays OFF until I say so! \"${_option_strings_first}\" is needed to enable ${_option}. Now stop bothering me, and install your dependencies, if you really want to enable this option.")
      endif()
      unset(${_option} CACHE)
    endif()
  endif()

  cmake_dependent_option(${_option} "${_doc}" ${_default} "${_deps}" ${_force})

  unset(_missing_deps)
  foreach(_dep ${_deps})
    string(REGEX REPLACE " +" ";" _depx "${_dep}")
    if(NOT (${_depx}))
      list(APPEND _missing_deps "${_dep}")
    endif()
  endforeach()

  if(DEFINED _missing_deps)
    set(${_option}_disable_reason " (dependencies unsatisfied: \"${_missing_deps}\")")
    # Set a value that can be visualized on ccmake and on cmake-gui, but
    # still evaluates to false
    set(${_option} "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND" CACHE STRING "${_option_doc}" FORCE)
    string(REPLACE ";" "\;" _missing_deps "${_missing_deps}")

    # Set non-cache variable that will override the value in current scope
    # For parent scopes, the "-NOTFOUND ensures that the variable still
    # evaluates to false
    set(${_option} ${_force})
  endif()

endmacro()
