#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


int main (int argc, char** argv)
{
    bool show_viewer = false;
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::string fname;

    if (argc == 2) {
	fname = argv[1];
    } else if (argc == 3 && strcmp(argv[1], "-v") == 0) {
	show_viewer = true;
	fname = argv[2];
    } else {
	std::cerr << "usage: " << argv[0] << " [-v]<file.pcd" << std::endl;
	exit(-1);
    }


    pcl::io::loadPCDFile(fname, *cloud);
    std::cerr << "Loaded " << cloud->points.size () << " data points from "<< fname << "." << std::endl;

    for (size_t i = 0; i < cloud->points.size (); ++i)
	std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

    if (show_viewer) {
	pcl::visualization::CloudViewer viewer ("PCD Viewer");

	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	    boost::this_thread::sleep (boost::posix_time::seconds (1));

    }
    return (0);
}
