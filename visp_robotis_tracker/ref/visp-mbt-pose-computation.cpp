#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>

int main()
{
  vpImage<unsigned char> I;
  
  //Image grabber initialisation
  vpOpenCVGrabber g ;
  g.setDeviceType (CV_CAP_ANY);
  g.open();
  g.acquire(I);
  
  //Display initialisation
  #if defined VISP_HAVE_X11
  vpDisplayX d;
  #elif defined VISP_HAVE_GDI
  vpDisplayGDI d;
  #elif defined VISP_HAVE_OPEN_CV
  vpDisplayOpenCV d;
  #endif
  d.init(I, 0, 0, "") ;
  
  vpDisplay::display(I);
  vpDisplay::flush(I);
  
  //Model-based tracker initialisation
  vpMbEdgeTracker tracker;
  tracker.setDisplayMovingEdges (true);
  vpHomogeneousMatrix cMo;
  
  tracker.loadConfigFile("cadre.xml");
  
  vpCameraParameters cam;
  tracker.getCameraParameters(cam);
  
  tracker.loadModel("cadre.wrl");
  
  tracker.initClick(I, "cadre", true);
  vpDisplay::flush(I);
  vpDisplay::getClick(I);
  tracker.track(I);
  
  //Pose computation
  tracker.getPose(cMo);
  
  while ( 1 )
  {
    g.acquire(I);

    // Track the model and get the pose.
    tracker.track(I);
    tracker.getPose(cMo);
    
    vpDisplay::display(I);
    
    //Display the model and the frame
    tracker.display(I,cMo,cam,vpColor::green);
    vpDisplay::displayFrame(I,cMo,cam,0.05,vpColor::none,2);
    
    vpDisplay::flush(I);
  }
  
  return 0;
}
