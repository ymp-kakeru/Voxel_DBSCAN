#include <iostream>

#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

#include <pcl/octree/octree_impl.h> //octreeの各種テンプレートの使用にはここのマクロ？が必要らしい。Densityだけっぽいけど
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/filter.h>
//#include "boost.h"

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>

//=============================
// Displaying cubes is very long!
// so we limit their numbers.
 const int MAX_DISPLAYED_CUBES(15000);
//=============================
  
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN>::Ptr PointCloudN;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL>::Ptr PointCloudL;

#define debug() std::cerr << __LINE__ << std::endl;

class VoxelDBSCAN
{
public:
  VoxelDBSCAN(std::string & FileName, double resolution_tree);
  bool loadPCD(std::string filename);
  void GenerateOctree();
  void dbscan();
  void extractPointsAtLevel(int depth);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;

  PointCloudT cloud_in;
  pcl::octree::OctreePointCloudDensity<PointT> octree;

  int displayedDepth ;
  bool displayCubes, showPointsWithCubes, wireframe;

};

/*****************************************************************************************************/
VoxelDBSCAN::VoxelDBSCAN(std::string & FileName, double resolution_tree):
  octree(resolution_tree),cloud_in (new pcl::PointCloud<pcl::PointXYZ>()),
  {
  if(!loadPCD(FileName))
    return;
  GenerateOctree();
  //set current level to half the maximum one
  //show octree at default depth
  extractPointsAtLevel(octree.getTreeDepth());

  //run main loop
  run();

}

/***********************************************************************************************************/
bool VoxelDBSCAN::loadPCD(std::string filename)
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

/*******************************************************************************************************/
void VoxelDBSCAN::GenerateOctree()
{
  /* make octree  */
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();

  /* octree leaf iterator */
  std::vector<int> indexVector;
  pcl::PointCloud<PointT>::iterator it;
  pcl::octree::OctreePointCloudDensity<PointT>::LeafNodeIterator leaf_it(&octree);

}

/**********************************************************************************************************/
void VoxelDBSCAN:: extractPointsAtLevel(int depth)
{
  pcl::octree::OctreePointCloudDensity<PointT>::Iterator tree_it;
  pcl::octree::OctreePointCloudDensity<PointT>::Iterator tree_it_end = octree.end();

  pcl::PointXYZ pt;
  std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
  double start = pcl::getTime ();

  for (tree_it = octree.begin(depth); tree_it!=tree_it_end; ++tree_it)
  {
    Eigen::Vector3f voxel_min, voxel_max;
    octree.getVoxelBounds(tree_it, voxel_min, voxel_max);

    pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
    pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
    pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;

    if(octree.getVoxelDensityAtPoint(pt) > 20)
    {
    std::cout <<  "get voxel density : " << pt.x << ","
                                         << pt.y << ","
                                         << pt.z << ","
                                         << octree.getVoxelDensityAtPoint(pt)
                                         << std::endl;
    }
//    std::cout <<  "get voxel density : " << octree.getVoxelDensityAtPoint(tree_it) << std::endl;
  }

  double end = pcl::getTime ();
  printf("%.4gs.=====\n",end - start);
}
 
int main(int argc, char const *argv[])
{
  /* code */
  std::string pcd_path(argv[1]);
  VoxelDBSCAN dbscan(pcd_path, atof(argv[2]));
  return 0;
}