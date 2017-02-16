/*
 * main.cpp
 *
 *  Created on: Jun 22, 2016
 *      Author: robotis
 */

#include "visp_robotis_tracker/node.h"

int main(int argc,char** argv){
  ros::init(argc, argv, "visp_robotis_tracker");
  visp_robotis_tracker::Node().spin();
  return 0;
}



