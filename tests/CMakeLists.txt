# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_liegroupcontrollers_test(
  NAME ProportionalController
  SOURCES ProportionalControllerTest.cpp
  LINKS LieGroupControllers::LieGroupControllers)

add_liegroupcontrollers_test(
  NAME ProportionalDerivativeController
  SOURCES ProportionalDerivativeControllerTest.cpp
  LINKS LieGroupControllers::LieGroupControllers)
