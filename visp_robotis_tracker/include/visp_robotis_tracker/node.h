/*
 * node.h
 *
 *  Created on: Jun 22, 2016
 *      Author: robotis
 */

#ifndef SRC_VISP_ROBOTIS_TRACKER_INCLUDE_VISP_ROBOTIS_TRACKER_NODE_H_
#define SRC_VISP_ROBOTIS_TRACKER_INCLUDE_VISP_ROBOTIS_TRACKER_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <visp_ros/vpROSGrabber.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>

namespace visp_robotis_tracker
{

class Node
{
private:
  ros::NodeHandle nh_;
  std::string image_raw_path_;
  std::string camera_info_path_;

  std::string package_path_;
  std::string model_path_;
  std::string object_name_;

//  vpImage<vpRGBa> img_; // Create a RGB level image container
  vpImage<unsigned char> img_; // Create a gray level image container
  vpROSGrabber grab_; // Create a grabber based on ROS

public:
  Node();
  void spin();
};

}

#endif /* SRC_VISP_ROBOTIS_TRACKER_INCLUDE_VISP_ROBOTIS_TRACKER_NODE_H_ */
