#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMbEdgeTracker.h>
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

  vpMbEdgeTracker tracker;
  vpHomogeneousMatrix cMo;

  // Load tracker config file (camera parameters and moving edge settings)
  tracker.loadConfigFile("mmicro.xml");

  // initialise an instance of vpCameraParameters with the parameters from the tracker
  vpCameraParameters cam;
  tracker.getCameraParameters(cam);
  
  // Load the 3D model (either a vrml file or a .cao file)
  tracker.loadModel("mmicro.wrl");
  
  //Initialize the tracking.
  tracker.initClick(I, "mmicro", true);

  //track the model
  tracker.track(I);
  tracker.getPose(cMo);
  tracker.display(I, cMo, cam, vpColor::red, 1);
  vpDisplay::flush(I);
  
  while ( 1 )
  {
    g.acquire(I);
    vpDisplay::display(I);

    //Track the object.
    tracker.track(I);
    tracker.getPose(cMo);
    tracker.display(I, cMo, cam, vpColor::red, 1);
    vpDisplay::displayFrame (I, cMo, cam, 0.05, vpColor::blue);
    vpDisplay::flush(I);
  }
  
  return 0;
}