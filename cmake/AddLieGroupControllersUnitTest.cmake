# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(FetchContent)
FetchContent_Declare(Catch2
                    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
                    GIT_TAG        v2.13.8)

FetchContent_GetProperties(Catch2)
if(NOT Catch2_POPULATED)
  message(STATUS "Fetching Catch2...")
  FetchContent_MakeAvailable(Catch2)
endif()


liegroupcontrollers_dependent_option(LIEGROUPCONTROLLERS_RUN_Valgrind_tests
  "Run Valgrind tests?" OFF
  "BUILD_TESTING;VALGRIND_FOUND" OFF)

if (LIEGROUPCONTROLLERS_RUN_Valgrind_tests)
    set(CTEST_MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
    set(MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
    set(MEMORYCHECK_COMMAND_OPTIONS "--leak-check=full --error-exitcode=1 ${MEMORYCHECK_SUPPRESSIONS}"  CACHE STRING "Options to pass to the memory checker")
    mark_as_advanced(MEMORYCHECK_COMMAND_OPTIONS)
    set(MEMCHECK_COMMAND_COMPLETE "${MEMORYCHECK_COMMAND} ${MEMORYCHECK_COMMAND_OPTIONS}")
    separate_arguments(MEMCHECK_COMMAND_COMPLETE)
endif()

if (BUILD_TESTING)
    configure_file(cmake/Catch2Main.cpp.in ${CMAKE_BINARY_DIR}/Testing/Catch2Main.cpp)
    add_library(CatchTestMain ${CMAKE_BINARY_DIR}/Testing/Catch2Main.cpp)
    target_link_libraries(CatchTestMain PRIVATE Catch2::Catch2)
endif()


function(add_liegroupcontrollers_test)

    if(BUILD_TESTING)

      set(options)
      set(oneValueArgs NAME)
      set(multiValueArgs SOURCES LINKS COMPILE_DEFINITIONS)

      set(prefix "lie_group_controllers")

      cmake_parse_arguments(${prefix}
          "${options}"
          "${oneValueArgs}"
          "${multiValueArgs}"
          ${ARGN})

      set(name ${${prefix}_NAME})
      set(unit_test_files ${${prefix}_SOURCES})

      set(targetname ${name}UnitTests)
      add_executable(${targetname}
          "${unit_test_files}")

      target_link_libraries(${targetname} PRIVATE CatchTestMain ${${prefix}_LINKS} Catch2::Catch2)
      target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
      target_compile_features(${targetname} PUBLIC cxx_std_14)

      add_test(NAME ${targetname} COMMAND ${targetname})
      target_compile_definitions(${targetname} PRIVATE ${${prefix}_COMPILE_DEFINITIONS})

      if(LIEGROUPCONTROLLERS_RUN_Valgrind_tests)
        add_test(NAME memcheck_${targetname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${targetname}>)
      endif()

    endif()

endfunction()
