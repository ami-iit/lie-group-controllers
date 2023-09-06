#.rst:
# AddUninstallTarget
# ------------------
#
# Add the "uninstall" target for your project::
#
#   include(AddUninstallTarget)
#
#
# will create a file cmake_uninstall.cmake in the build directory and add a
# custom target uninstall that will remove the files installed by your package
# (using install_manifest.txt)

# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


if(DEFINED __ADD_UNINSTALL_TARGET_INCLUDED)
  return()
endif()
set(__ADD_UNINSTALL_TARGET_INCLUDED TRUE)


set(_filename ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

file(WRITE ${_filename}
  "if(NOT EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\")
    message(WARNING \"Cannot find install manifest: \\\"${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\\\"\")
    return()
endif()

file(READ \"${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\" files)
string(STRIP \"\${files}\" files)
string(REGEX REPLACE \"\\n\" \";\" files \"\${files}\")
list(REVERSE files)
foreach(file \${files})
    message(STATUS \"Uninstalling: \$ENV{DESTDIR}\${file}\")
    if(EXISTS \"\$ENV{DESTDIR}\${file}\")
        execute_process(
            COMMAND \${CMAKE_COMMAND} -E remove \"\$ENV{DESTDIR}\${file}\"
            OUTPUT_VARIABLE rm_out
            RESULT_VARIABLE rm_retval)
        if(NOT \"\${rm_retval}\" EQUAL 0)
            message(FATAL_ERROR \"Problem when removing \\\"\$ENV{DESTDIR}\${file}\\\"\")
        endif()
    else()
        message(STATUS \"File \\\"\$ENV{DESTDIR}\${file}\\\" does not exist.\")
    endif()
endforeach(file)
")

if("${CMAKE_GENERATOR}" MATCHES "^(Visual Studio|Xcode)")
  set(_uninstall "UNINSTALL")
else()
  set(_uninstall "uninstall")
endif()
add_custom_target(${_uninstall} COMMAND "${CMAKE_COMMAND}" -P "${_filename}")
set_property(TARGET ${_uninstall} PROPERTY FOLDER "CMakePredefinedTargets")
