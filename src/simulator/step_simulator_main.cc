//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 * \file    simulator_main.cpp
 * \brief   A simple simulator.
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>

#include <iostream>

#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"


#include "util/timer.h"
#include "simulator.h"

DEFINE_double(fps, 120.0, "Simulator frames rate.");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  printf("\nUT AUTOmata F1/10 Simulator\n\n");

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> n = std::make_shared<rclcpp::Node>("ut_automata_simulator");

  Simulator simulator;
  simulator.Init(n);
  simulator.SetStepMode(true);

  // main loop
  RateLoop rate(FLAGS_fps);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(n);
  while (rclcpp::ok()){
    executor.spin_once();
    // Wait for a character to be pressed.
    getchar();
    simulator.RunIteration();
    rate.Sleep();
  }

  printf("closing.\n");

  return(0);
}
