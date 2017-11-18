#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni2_grabber.h>


typedef pcl::PointXYZ PT;
//typedef pcl::PointXYZRGBA PT;

class SimpleOpenNIProcessor
{
public:
  void write_frame (const pcl::PointCloud<PT>::ConstPtr &cloud)
  {
    static unsigned frame = 0;
    std::stringstream fname;
    fname << "frame-" << frame << ".pcd";
    std::cout << "Writing cloud to " << fname.str() << std::endl;
    pcl::io::savePCDFileASCII (fname.str(), *cloud);
    frame++;
  }
  void cloud_cb_ (const pcl::PointCloud<PT>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static double last = pcl::getTime ();
    if (++count == 30)
    {
      double now = pcl::getTime ();
      std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
      write_frame(cloud);
    }
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    auto* interface = new pcl::OpenNIGrabber();

    std::cout << "OpenNI device " << interface->getDevice()->getConnectionString() << std::endl;

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<PT>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}
