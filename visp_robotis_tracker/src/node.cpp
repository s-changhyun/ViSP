/*
 * node.cpp
 *
 *  Created on: Jun 22, 2016
 *      Author: robotis
 */

#include "visp_robotis_tracker/node.h"
#include "visp_robotis_tracker/names.h"

namespace visp_robotis_tracker
{

Node::Node() :
    nh_(""),
    image_raw_path_(),
    camera_info_path_()
{
  image_raw_path_ = nh_.param<std::string>("image_raw","");
  camera_info_path_ = nh_.param<std::string>("camera_info","");

  object_name_ = nh_.param<std::string>("object_name","");

  package_path_ = ros::package::getPath("visp_robotis_tracker");
  model_path_ = package_path_ + "/model/";
}

void Node::spin()
{
  int opt_tracker = 0;

  try
  {
    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;

    grab_.setImageTopic(image_raw_path_); // Setting camera topic
    grab_.setCameraInfoTopic(camera_info_path_); // Setting camera info

    grab_.setRectify(true); // Turn auto shutter on
    grab_.open(img_); // Opening

    ROS_INFO("Image size: %d , %d", img_.getWidth(), img_.getHeight());

    vpDisplay *display = NULL;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#else
    display = new vpDisplayOpenCV;
#endif
    display->init(img_, 100, 100, "Model-based tracker");

    vpMbTracker *tracker;
    if (opt_tracker == 0)
     tracker = new vpMbEdgeTracker;
#ifdef VISP_HAVE_MODULE_KLT
    else if (opt_tracker == 1)
      tracker = new vpMbKltTracker;
    else
      tracker = new vpMbEdgeKltTracker;
#else
    else
    {
      ROS_INFO("klt and hybrid model-based tracker are not available since visp_klt module is missing");
      return 0;
    }
#endif

    if (opt_tracker == 0 || opt_tracker == 2)
    {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
    }

#ifdef VISP_HAVE_MODULE_KLT
    if (opt_tracker == 1 || opt_tracker == 2)
    {
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      dynamic_cast<vpMbKltTracker*>(tracker)->setKltOpencv(klt_settings);
      dynamic_cast<vpMbKltTracker*>(tracker)->setMaskBorder(5);
    }
#endif

    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    tracker->setCameraParameters(cam);
    tracker->loadModel(model_path_ + object_name_ + ".wrl");
    tracker->setDisplayFeatures(true);
    tracker->initClick(img_, model_path_ + object_name_ + ".init", true);

    while(ros::ok())
    {
      grab_.acquire(img_);
      vpDisplay::display(img_);
      tracker->track(img_);
      tracker->getPose(cMo);
      tracker->getCameraParameters(cam);
      tracker->display(img_, cMo, cam, vpColor::red, 2, true);
      vpDisplay::displayFrame(img_, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(img_, 20, 20, "A click to exit...", vpColor::red);
      vpDisplay::flush(img_);
      if (vpDisplay::getClick(img_, false))
        break;
    }
    vpDisplay::getClick(img_);
    delete display;
    delete tracker;
  }
  catch(vpException &e)
  {
    ROS_INFO_STREAM("Catch an exception" << e.getStringMessage());
  }
}

}
