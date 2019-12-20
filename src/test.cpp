#include <iostream>
#include <cmath>

#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/filter.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
int
main (int argc, char *argv[])
{
  srand (time (NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
/*  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }*/

  std::string filename(argv[1]) ;

  if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename,*cloud) == -1)
  {
    std::cerr << "ERROR : Counldn't read PCD file " << std::endl;
    return false;
  }
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; 

  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;
  std::cout << "get PointCloud" << std::endl;

  for(std::size_t j=0; j <= cloud->points.size(); j+=100)
  {
    double distance = sqrt(pow(cloud->points[j].x,2)+pow(cloud->points[j].y,2)+pow(cloud->points[j].z,2));

    std::cout << cloud->points[j].x << ","
              << cloud->points[j].y << ","
              << cloud->points[j].z << "       " << distance << std::endl ;
  } 

  return 0;
}
