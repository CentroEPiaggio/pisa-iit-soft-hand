/** @file iit_hand_hw_node.cpp
 *  @brief A simple ROS node implementing a ros_control loop for the Pisa/IIT
 * softhand.
 *  @author Murilo Martins (murilo.martins@ocado.com)
 *
 * Copyright (c) 2016, Ocado Technology
 * All rights reserved.
 *
 * This file is part of soft_hand_ros_control:
 * https://github.com/CentroEPiaggio/pisa-iit-soft-hand/tree/master/soft_hand_ros_control
 *
 * soft_hand_ros_control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.

 * soft_hand_ros_control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with soft_hand_ros_control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "soft_hand_ros_control/iit_hand_hw.h"

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

constexpr int CTRL_FREQ = 1000; // Hz

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "iit_softhand_hw", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // construct the Pisa/IIT softhand
  ros::NodeHandle iit_nh;
  iit_hand_hw::IITSH_HW iit_softhand;

  ros::Rate loop_rate(CTRL_FREQ);

  // initialisation/configuration routine
  iit_softhand.init(iit_nh, iit_nh);

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  //the controller manager
  controller_manager::ControllerManager manager(&iit_softhand, iit_nh);

  while (!g_quit) {
    // get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    }
    else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }

    // read the current state of the iit softhand
    iit_softhand.read(now, period);

    // update the controllers
    manager.update(now, period);

    // write the command to the iit softhand
    iit_softhand.write(now, period);

    loop_rate.sleep();
  }

  std::cerr <<" Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Done!" << std::endl;

  return 0;
}
