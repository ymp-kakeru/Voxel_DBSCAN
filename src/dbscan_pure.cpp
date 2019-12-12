#include <iostream>

#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/filter.h>
//#include "boost.h"

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN>::Ptr PointCloudN;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL>::Ptr PointCloudL;

#define debug() std::cerr << __LINE__ << std::endl;

class DBSCAN
{
public:
  DBSCAN(std::string & FileName, double resolution_tree);
  bool loadPCD(std::string filename);
  void dbscan();

private:
  void EpsCount(float radius); //count point-number in the radius from point[i]
  PointCloudT cloud_in;
  PointCloudT cloud_out;
  PointCloudT point_now;
  PointCloudT point_old;

  std::vector<pcl::PointIndices> cluster_indices;
  int minpts; // 
  float eps; //radius [m]

};

/*****************************************************************************************************/
DBSCAN::DBSCAN(std::string & FileName):
  eps(0.05),minpts(2000)
{
  if(!loadPCD(FileName))
    return;
  minpts = sqrt(cloud_in.size());
//run main loop
  run();
}

/***********************************************************************************************************/
bool DBSCAN::loadPCD(std::string filename)
{
  /* include point cloud data */
  if(pcl::io::loadPCDFile<PointT>(filename,*cloud_in) == -1)
  {
    std::cerr << "ERROR : Counldn't read PCD file " << std::endl;
    return false;
  }
  std::cout << "PointCloud before filtering has: " << cloud_in->points.size () << " data points." << std::endl; 

  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, nanIndexes);
  std::cout << "Loaded " << cloud_in->points.size() << " points" << std::endl;
  std::cout << "get PointCloud" << std::endl;
  return true;
}
/**************************************************************************************************/
void DBSCAN::EpsCount(float radius)
{
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud_in);
  //K nearest neighbor search K近傍探索
  std::vector<int> pointIdxNRNSearch(radius);
  std::vector<float> pointNRNSquaredDistance(radius);

  if(kdtree.radiusSearch(point_now,radius,pointIdxNRNSearch,pointNRNSquaredDistance)>0)
  {
    std::cout << "size in eps :" << pointIdxNRNSearch.size() << std::endl;
    return pointIdxNRNSearch.size() ;
  }
  return 0;
}
/****************************************************************************************/

void DBSCAN::dbscan()
{
  for(std::size_t i=0; i <= cloud_in.size(); ++i)
  {
    DBSCAN::point_now = cloud_in[i] ;
    if( EpsCount(eps) >= minpts )
    {
      
    }
  }
}
 
/*****************************************************************************************/
int main(int argc, char const *argv[])
{
  /* code */
  std::string pcd_path(argv[1]);
  DBSCAN dbscan(pcd_path, atof(argv[2]));
  return 0;
}