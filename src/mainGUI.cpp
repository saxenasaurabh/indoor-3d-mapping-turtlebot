/**
 * A software for RGBD maps of indoor environments
 * Sample: ./mainGUI -minps 100 -nordw 0.5 -disth 0.09 -iter 100 -nerad 0.11 -mincs 100 -ctol 0.07 -histcomp 3 -icpiter 0 -leaf 0.04 -lferad 0.1 -saciter 10 -sacmnsd 0.08 -mlsrad 0.1 -mlspol 0 -planeth 2.5 -zlimit 4.0 -coeffwt 2.0 -maxdist 0.05 -saccycles 1 -saccw 0.0 -deltar 5 -deltag 5 -deltab 5 -kdrad 0.2 -clusterth 2 -anchors 1 -useodom 1 -useplanes 1 -useclusters 1
 */
#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include "pairwiseRegistration.h"

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

void
printHelp (int, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_info ("Usage: %s [options]\n", argv[0]);
  print_info ("-h | --help : shows this help\n");
  print_info ("-minps : Minimum Plane size\n");
  print_info ("-mincs : Minimum Cluster size\n");
  print_info ("-nordw : Normal Distance Weight\n");
  print_info ("-disth : Distance Threshold for plane estimation using RANSAC\n");
  print_info ("-iter : Max RANSAC iterations for plane estimation\n");
  print_info ("-nerad : Normal Estimation Radius\n");
  print_info ("-ctol : Cluster Tolerance\n");
  print_info ("-maxcs : Max Cluster size\n");
  print_info ("-histcomp : Histogram Comparison Method\n");
  print_info ("-icpiter : No. of ICP iterations\n");
  print_info ("-psize : Point size for visualizer\n");
  print_info ("-leaf : Leaf size for MLS smoothing\n");
  print_info ("-visualize : Visualization of intermediate state, all clusters, planes and their histograms and match scores\n");
  print_info ("-lferad : Local Feature Estimation search radius\n");
  print_info ("-saciter : Max SACIA iterations\n");
  print_info ("-sacmnsd : SACIA minimum sample distance\n");
  print_info ("-anchors : Max number of clusters to include in one anchor\n");
  print_info ("-zlimit : Z-limit for passthrough filter\n");
  print_info ("-mlsrad : MLS smoothing radius\n");
  print_info ("-mlspol : 0/1 Set MLS polynomial fit off/on\n");
  print_info ("-planeth : Max threshold for plane histogram comparison score\n");
  print_info ("-clusterth : Max threshold for cluster histogram comparison score\n");
  print_info ("-coeffwt : Weight given to coefficients while comparing planes\n");
  print_info ("-maxdist : Max distance between points for fitness score computation\n");
  print_info ("-saccycles : Max number of SAC cycles\n");
  print_info ("-saccw : Weight given to constraints while computing fitness score\n");
  print_info ("-ksearch : Search parameter for Kd-Tree\n");
  print_info ("-deltar : Threshold for R while comparing points for fitness score\n");
  print_info ("-deltag : Threshold for G while comparing points for fitness score\n");
  print_info ("-deltab : Threshold for B while comparing points for fitness score\n");
  print_info ("-kdrad : Kd-Tree search radius\n");
  print_info ("-useodom : 0/1 Flag for Turtlebot odometry\n");
  print_info ("-useplanes : 0/1 Flag for turning off/on alignment using planes\n");
  print_info ("-useclusters : 0/1 Flag for turning off/on alignment using clusters\n");
  print_info ("-fswad : Fitness Score Weight of Average Distance\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class MainWindow
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename Cloud::Ptr CloudPtr;

    MainWindow (pcl::Grabber& grabber, int argc, char** argv)
      : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Main Window"))
      , image_viewer_ ()
      , grabber_ (grabber)
      , rgb_data_ (0), rgb_data_size_ (0)
      , v1 (0), v2 (0)
      , cloud_ (new Cloud)
      , seeded (false)
      , cloud_modified (false)
      , index (1)
    { 
      int temp;
      float tempf;
      
      if(pcl::console::find_argument(argc, argv, "-minps")!=-1) // Min plane size
      {
	pcl::console::parse_argument (argc, argv, "-minps", temp);
	reg.setMinPlaneSize(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-mincs")!=-1) // Min cluster size
      {
	pcl::console::parse_argument (argc, argv, "-mincs", temp);
	reg.setMinClusterSize(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-nordw")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-nordw", tempf);
	reg.setNormalDistanceWeight(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-disth")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-disth", tempf);
	reg.setDistanceThreshold(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-iter")!=-1) 
      {
	pcl::console::parse_argument (argc, argv, "-iter", temp);
	reg.setMaxRANSACIterations(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-nerad")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-nerad", tempf);
	reg.setNormalEstimationRadius(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-ctol")!=-1) 
      {
	pcl::console::parse_argument (argc, argv, "-ctol", tempf);
	reg.setClusterTolerance(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-maxcs")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-maxcs", temp);
	reg.setMaxClusterSize(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-histcomp")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-histcomp", temp);
	if(!reg.isValidHistComparisonMethod(temp))
	{
	  printf("Invalid value for -histcomp...Using default value\n");
	}
	else
	{
	  reg.setHistComparisonMethod(temp);
	}
      }
      
      if(pcl::console::find_argument(argc, argv, "-icpiter")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-icpiter", temp);
	reg.setMaxICPIterations(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-psize")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-psize", temp);
	reg.setPointSize(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-leaf")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-leaf", tempf);
	reg.setLeafSize(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-visualize")!=-1)
      {
	reg.setVisualize(true);
      }
      
      if(pcl::console::find_argument(argc, argv, "-lferad")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-lferad", tempf);
	reg.setLocalFeatureRadius(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-saciter")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-saciter", temp);
	reg.setSACIAIterations(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-sacmnsd")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-sacmnsd", tempf);
	reg.setSACIAMinSampleDistance(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-anchors")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-anchors", temp);
	reg.setNumAnchors(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-zlimit")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-zlimit", tempf);
	reg.setZLimit(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-mlsrad")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-mlsrad", tempf);
	reg.setMLSSearchRadius(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-mlspol")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-mlspol", temp);
	reg.setMLSPolynomialFitFlag(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-planeth")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-planeth", tempf);
	reg.setPlaneMatchThreshold(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-clusterth")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-clusterth", tempf);
	reg.setClusterMatchThreshold(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-coeffwt")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-coeffwt", tempf);
	reg.setWeightOfCoeff(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-maxdist")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-maxdist", tempf);
	reg.setMaxDistForFitnessScore(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-saccycles")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-saccycles", temp);
	reg.setSACCycles(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-saccw")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-saccw", tempf);
	reg.setSACWeightOfConstraints(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-ksearch")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-ksearch", temp);
	reg.setKSearch(temp);
	reg.setKSearchFlag(true);
      }
      
      if(pcl::console::find_argument(argc, argv, "-deltar")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-deltar", temp);
	reg.setDeltaRed(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-deltag")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-deltag", temp);
	reg.setDeltaGreen(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-deltab")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-deltab", temp);
	reg.setDeltaBlue(temp);
      }
      
      if(pcl::console::find_argument(argc, argv, "-kdrad")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-kdrad", tempf);
	reg.setKDTreeSearchRadius(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-useodom")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-useodom", temp);
	if(temp)
	  reg.setUseTurtlebotOdometry(true);
	else
	  reg.setUseTurtlebotOdometry(false);
      }
      
      if(pcl::console::find_argument(argc, argv, "-useplanes")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-useplanes", temp);
	if(temp)
	  reg.setUsePlanes(true);
	else
	  reg.setUsePlanes(false);
      }
      
      if(pcl::console::find_argument(argc, argv, "-useclusters")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-useclusters", temp);
	if(temp)
	  reg.setUseClusters(true);
	else
	  reg.setUseClusters(false);
      }
      
      if(pcl::console::find_argument(argc, argv, "-fswad")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-fswad", tempf);
	reg.setWeightOfAvgDists(tempf);
      }
      
      if(pcl::console::find_argument(argc, argv, "-mls")!=-1)
      {
	pcl::console::parse_argument (argc, argv, "-mls", temp);
	if(temp)
	  reg.setMLSFlag(true);
	else
	  reg.setMLSFlag(false);
      }
    }
    
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      copyPointCloud<PointType> (*cloud, *cloud_);
      cloud_modified = true;
    }

    void
    image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
    {
      FPS_CALC ("image callback");
      boost::mutex::scoped_lock lock (image_mutex_);
      image_ = image;
      
      if (image->getEncoding () != openni_wrapper::Image::RGB)
      {
        if (rgb_data_size_ < image->getWidth () * image->getHeight ())
        {
          if (rgb_data_)
            delete [] rgb_data_;
          rgb_data_size_ = image->getWidth () * image->getHeight ();
          rgb_data_ = new unsigned char [rgb_data_size_ * 3];
        }
        image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
      }
    }
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if ( (event.getKeySym () == "c" || event.getKeySym () == "C") && event.keyDown ())
      {
	// Capture current frame
	
	if(grabber_.isRunning())
	{
	  grabber_.stop();
	}
	
	CloudPtr cloud (new Cloud);
	if (cloud_mutex_.try_lock ())
        {
          copyPointCloud<PointType> (*cloud_, *cloud);
          cloud_mutex_.unlock ();
        }

        if (!cloud->empty())
        {
	  /*if(!seeded)
	  {
	    reg.setSeedCloud (cloud);
	    seeded = !seeded;
	  }
	  else
	    */
	  reg.add (cloud);
	  CloudPtr registered = reg.getRegisteredCloud();
	  if (!cloud_viewer_->updatePointCloud (registered, "Registered"))
          {
            cloud_viewer_->addPointCloud (registered, "Registered", v2);
//             cloud_viewer_->resetCameraViewpoint ("Registered");
	  }
        }
        grabber_.start();
      }
      else if ( (event.getKeySym () == "z" || event.getKeySym () == "Z") && event.keyDown ())
      {
	// Undo previous add
	reg.dropPreviousCloud ();
	CloudPtr registered = reg.getRegisteredCloud();
	if (!cloud_viewer_->updatePointCloud (registered, "Registered"))
	{
	  cloud_viewer_->addPointCloud (registered, "Registered", v2);
	  cloud_viewer_->resetCameraViewpoint ("Registered");
	}
      }
      else if ( (event.getKeySym () == "s" || event.getKeySym () == "S") && event.keyDown ())
      {
	// Save registered cloud
	CloudPtr registered = reg.getRegisteredCloud();
	char filename[100];
	sprintf(filename, "cloud%d.pcd", index++);
	printf("Saving %s...\n", filename);
	pcl::io::savePCDFileASCII (filename, *registered);
      }
    }
    
    void 
    mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    /**
     * @brief starts the main loop
     */
    void
    run ()
    {
      cloud_viewer_->registerMouseCallback (&MainWindow::mouse_callback, *this);
      cloud_viewer_->registerKeyboardCallback(&MainWindow::keyboard_callback, *this);
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&MainWindow::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
      
      cloud_viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
      cloud_viewer_->setBackgroundColor (0, 0, 0, v1);
//       CloudPtr cloud1 (new Cloud);
//       pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(cloud1);
//       viewer.addPointCloud<PointType> (cloud1, rgb1, "Openni Grabber", v1);
  
      cloud_viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
      cloud_viewer_->setBackgroundColor (0, 0, 0, v2);
//       CloudPtr cloud2 (new Cloud);
//       pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(cloud2);
//       viewer.addPointCloud<PointType> (cloud2, rgb2, "Registered", v2);

//       cloud_viewer_->addCoordinateSystem (1.0);
//       cloud_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, reg.getPointSize(), "Openni Grabber");
//       cloud_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, reg.getPointSize(), "Registered");
//       cloud_viewer_->spin();
            
      boost::signals2::connection image_connection;
      if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>())
      {
        image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
        image_viewer_->registerMouseCallback (&MainWindow::mouse_callback, *this);
        image_viewer_->registerKeyboardCallback(&MainWindow::keyboard_callback, *this);
        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&MainWindow::image_callback, this, _1);
        image_connection = grabber_.registerCallback (image_cb);
      }
      
      bool image_init = false, cloud_init = false;
      
      grabber_.start ();

      while (!cloud_viewer_->wasStopped () && (image_viewer_ && !image_viewer_->wasStopped ()))
      {
        boost::shared_ptr<openni_wrapper::Image> image;
        CloudPtr cloud(new Cloud);

        cloud_viewer_->spinOnce ();

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
	  if(cloud_modified)
	  {
	    copyPointCloud<PointType> (*cloud_, *cloud);
	  }
          cloud_mutex_.unlock ();
        }

        if (cloud_modified && !cloud->empty())
        {
          FPS_CALC ("drawing cloud");
          
          if (!cloud_init)
          {
	    // Not present in PCL 1.6
//             cloud_viewer_->setPosition (0, 0);
//             cloud_viewer_->setSize (cloud->width, cloud->height);
            cloud_init = !cloud_init;
          }

          if (!cloud_viewer_->updatePointCloud (cloud, "Openni Grabber"))
          {
            cloud_viewer_->addPointCloud (cloud, "Openni Grabber", v1);
            cloud_viewer_->resetCameraViewpoint ("Openni Grabber");
          }
        }

        cloud_modified = false;
        
        // See if we can get an image
        if (image_mutex_.try_lock ())
        {
          image_.swap (image);
          image_mutex_.unlock ();
        }

        if (image)
        {
          if (!image_init && cloud && cloud->width != 0)
          {
            image_viewer_->setPosition (cloud->width, 0);
            image_viewer_->setSize (cloud->width, cloud->height);
            image_init = !image_init;
          }

          if (image->getEncoding() == openni_wrapper::Image::RGB)
            image_viewer_->addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
          else
            image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
          image_viewer_->spinOnce ();
        }
        
      }

      grabber_.stop ();
      
      cloud_connection.disconnect ();
      image_connection.disconnect ();
      if (rgb_data_)
        delete[] rgb_data_;
    }
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;
    
    CloudPtr cloud_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;
    
    RegistrationUsingClusters reg;
    int v1, v2;
    int argc;
    char **argv;
    bool seeded;
    bool cloud_modified;
    int index;
};

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

/* ---[ */
int
main (int argc, char** argv)
{
  if(pcl::console::find_argument(argc, argv, "-h")!=-1 || pcl::console::find_argument(argc, argv, "-help")!=-1)
  {
    printHelp(argc, argv);
    return 0;
  }
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  std::string device_id("");
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  bool xyz = false;
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  
  if (driver.getNumberDevices() > 0)
    cout << "Device Id not set, using first device." << endl;  
  
  unsigned mode;
  if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
    depth_mode = pcl::OpenNIGrabber::Mode (mode);

  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    image_mode = pcl::OpenNIGrabber::Mode (mode);
  
  if (pcl::console::find_argument (argc, argv, "-xyz") != -1)
    xyz = true;
  
  pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  
//   if (xyz || !grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
//   {
//     MainWindow<pcl::PointXYZ> openni_viewer (grabber, argc, argv);
//     openni_viewer.run ();
//   }
//   else
//   {
    MainWindow<pcl::PointXYZRGB> openni_viewer (grabber, argc, argv);
    openni_viewer.run ();
//   }
  
  return (0);
}
/* ]--- */
