#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/time.h>

 #include <pcl/io/openni_grabber.h>
 #include <pcl/io/openni2_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       static unsigned count = 0;
       static unsigned frame = 0;
       static double last = pcl::getTime ();
       if (++count == 30)
       {
         double now = pcl::getTime ();
         std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
         count = 0;
         last = now;
         std::stringstream fname;
         fname << "frame-" << frame << ".pcd";
         std::cout << "Writing clould to " << fname.str() << std::endl;
         pcl::io::savePCDFileASCII (fname.str(), *cloud);
         frame++;
       }

//       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }
     
     pcl::visualization::CloudViewer viewer;
};

int main (int argc, char *argv[])
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
