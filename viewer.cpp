#include <iostream>

#include "libfreenect.h"
#include "libfreenect_sync.h"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;

#undef USE_RGB

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
}

#ifdef USE_RGB
typedef PointXYZRGB _PT_;
#else
typedef PointXYZ _PT_;
#endif

typedef PointCloud<_PT_> _CLOUD_;


const float X_RES = 640.0;
const float Y_RES = 480.0;
const float Z_RES = 640.0;
const float NAN_PT = std::numeric_limits<float>::quiet_NaN();

void write_frame (const _CLOUD_::ConstPtr &cloud)
{
    static unsigned frame = 0;
    std::stringstream fname;
    fname << "frame-" << frame << ".pcd";
    std::cout << "Writing cloud to " << fname.str() << std::endl;
    io::savePCDFileASCII (fname.str(), *cloud);
    frame++;
}



float fx = 594.21f;
float fy = 591.04f;
float a = -0.0030711f;
float b = 3.3309495f;
float cx = 339.5f;
float cy = 242.7f;

#if 0
float cx = 254.878f;
float cy = 205.395f;
float fx = 365.456f;
float fy = 365.456f;
#endif
const float KINECT_TRANSFORM[4][4] = {
    {   1/fx,     0,    0, 0},
    {   0,    -1/fy,    0, 0},
    {   0,        0,    0, a},
    {   -cx/fx, cy/fy, -1, b},
};

float rawDepthToMeters(int depthValue)
{
    static float* depthLookup = NULL;

    if (!depthLookup) {
        depthLookup = new float[2048];
        for (unsigned i =0; i < 2048; i++)
            depthLookup[i] = (float)(1.0 / ((double)(i) * -0.0030711016 + 3.3309495161));
    }

    if (depthValue < 2047)
        return depthLookup[depthValue];

    return NAN_PT;
}

_PT_ KinectToWorld(unsigned x, unsigned y, unsigned d)
{
    _PT_ p;

#if 0
    p.x = ((float)x-(X_RES/2.0));
    p.y = ((float)y-(Y_RES/2.0));
    //p.z = ((float)z-(Z_RES));
    p.z = (float)(1.0 / ((double)(d) * -0.0030711016 + 3.3309495161));
#endif
#if 0
    p.x = x;
    p.y = y;
    p.z = z;
#endif
#if 1
    const double fx_d = 1.0 / 5.9421434211923247e+02;
    const double fy_d = 1.0 / 5.9104053696870778e+02;
    const double cx_d = 3.3930780975300314e+02;
    const double cy_d = 2.4273913761751615e+02;

    // Drawing the result vector to give each point its three-dimensional
    // space
    double depth = rawDepthToMeters(d);
    p.x = (float)(((double)x - cx_d) * depth * fx_d);
    p.y = (float)(((double)y - cy_d) * depth * fy_d);
    p.z = (float)(depth);
#endif

    return p;
}

class Kinect {
   public:

      Kinect(const char* n)
         : name(n)
         , count(0)
         , depth (NULL)
#ifdef USE_RGB
         , rgb(NULL)
#endif
      {
         std::cout << "Connecting to kinect[" << name << "]:";
         for (int i = 0; i < 10; i++)
            if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
               no_kinect_quit();
         std::cout << std::endl;

         input_cloud = boost::make_shared<_CLOUD_>(640,480);
         output_cloud = boost::make_shared<_CLOUD_>();

         transform.setIdentity();
         for (int i = 0; i < 4; i++)
             for (int j = 0; j < 4; j++) {
                 std::cout << "(" << i << "," << j <<") = " << KINECT_TRANSFORM[i][j] << std::endl;
                 transform (i, j) = KINECT_TRANSFORM[j][i];
             }

         std::cout << transform << std::endl;
      };

      ~Kinect()
      {
#if 0
         free(depth);
#ifdef USE_RGB
         free(rgb);
#endif
#endif
         freenect_sync_stop();
      }

      const _CLOUD_::ConstPtr get_cloud()
      {
         if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
            no_kinect_quit();
#ifdef USE_RGB
          if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
              no_kinect_quit();
#endif
          for (unsigned x = 0; x < 640; x++) {
             for (unsigned y = 0; y < 480; y++) {
                unsigned offset = y*640+x;
                unsigned d = depth[offset];

                auto &p = input_cloud->at(x, y);

                if (1 && d == 2047) {
                   p.x = p.y = p.z = NAN_PT;
                } else {
                   p = KinectToWorld(x,y,d);
                }
#ifdef USE_RGB
                p.r = rgb[offset].r;
                p.g = rgb[offset].g;
                p.b = rgb[offset].b;
#endif
             }
          }

#if 0
          transformPointCloud(*input_cloud, *output_cloud, transform);
#else
          copyPointCloud(*input_cloud, *output_cloud);
#endif

            if (++count == 300) {
                write_frame(input_cloud);
                write_frame(output_cloud);
                count = 0;
            }

            return output_cloud;
        }

    protected:
        struct rgb {
            char r;
            char g;
            char b;
        };


        std::string name;
        unsigned count;
        uint32_t ts;
        short *depth;
#ifdef USE_RGB
        struct rgb *rgb;
#endif
        _CLOUD_::Ptr input_cloud;
        _CLOUD_::Ptr output_cloud;

        Eigen::Matrix4f transform;
};
 class SimpleOpenNIViewer
 {
    public:
        SimpleOpenNIViewer (Kinect *k) : kinect(k), viewer ("PCL OpenNI Viewer") {}

        void run ()
        {
            while (!viewer.wasStopped())
            {
                auto cloud = kinect->get_cloud();

                viewer.showCloud(cloud);
            }
        }
             
    protected:
        Kinect *kinect;
        visualization::CloudViewer viewer;
};


int main (int argc, char *argv[])
{
   Kinect kinect("usb");
   SimpleOpenNIViewer v(&kinect);
   v.run ();
   return 0;
}
