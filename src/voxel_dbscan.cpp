#include <iostream>

#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/cloud_iterator.h> 

#include <pcl/octree/octree_impl.h> //octreeの各種テンプレートの使用にはここのマクロ？が必要らしい。Densityだけっぽいけど
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
  
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
  bool extractDENSITY(PointT pt);
  bool extractCluster(PointCloudT inputCloud, PointCloudT outputCloud);
  void searchAdjacent(PointT pt,std::vector<PointT> it);
  void run();

  /*pcl viewer functions*/
//  void viewerOneOff(PointCloudT displayCloud);

private:
  PointCloudT cloud_in ;
  PointCloudT cloud_out ;
  PointCloudT octree_centroid_pts;
  PointT current;
  pcl::octree::OctreePointCloudDensity<PointT> octree;

  double minpts ,resolution;
  int min_cluster, max_cluster;
  Eigen::Vector3f voxel_min, voxel_max;

};
/*****************************************************************************************************/
VoxelDBSCAN::VoxelDBSCAN(std::string & FileName, double resolution_tree):
  octree(resolution_tree),cloud_in (new pcl::PointCloud<pcl::PointXYZ>()),cloud_out (new pcl::PointCloud<pcl::PointXYZ>()),
  octree_centroid_pts (new pcl::PointCloud<pcl::PointXYZ>()),minpts{0},resolution{resolution_tree},
  min_cluster{100},max_cluster{3000}
{
  if(!loadPCD(FileName))  
    return;
  //run main loop
  run();

}
/***********************************************************************************************************/
bool VoxelDBSCAN::loadPCD(std::string filename)
{
  /* include point cloud data */
  debug();
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
  minpts = pow(cloud_in->points.size(),0.33);
    std::cout << "=======" << "get PointCloud" << "," << "minpts =" << minpts << std::endl;
  return true;
}
/*******************************************************************************************************/
void VoxelDBSCAN::GenerateOctree()
{
  /* make octree  */
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();

  pcl::octree::OctreePointCloudDensity<PointT>::Iterator tree_it;
  PointT pt;
  for(tree_it = octree.leaf_begin(); tree_it != octree.leaf_end(); ++tree_it)
  {
    octree.getVoxelBounds(tree_it,voxel_min,voxel_max);
    pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
    pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
    pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
    octree_centroid_pts->points.push_back(pt);
  }
  std::cout << "Number of Voxels : " << octree.getLeafCount() << std::endl ;

}
/**********************************************************************************************************/
void VoxelDBSCAN:: run()
{
  GenerateOctree();
  while(1)
  {
    if(extractCluster(octree_centroid_pts, cloud_out)==true) break;
  }
//  viewerOneOff(cloud_out);
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("cluster viewer"));
  viewer->setBackgroundColor(0,0,0);
  viewer->addPointCloud<PointT>(cloud_out,"cluster_view");
  viewer->initCameraParameters();
  while (!viewer->wasStopped ())
  {
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  viewer->removePointCloud("cloud_out");
}
/**********************************************************************************************************/
bool VoxelDBSCAN:: extractDENSITY(PointT pt)
{
//  std::cout << "===== Extracting Density "<< "... " << std::endl;
  double start = pcl::getTime ();
  if(octree.getVoxelDensityAtPoint(pt) >= minpts)
  {
    return true;
  }
  return false;
//    std::cout <<  "get voxel density : " << octree.getVoxelDensityAtPoint(tree_it) << std::endl;
}
/**********************************************************************************************************/
bool VoxelDBSCAN::extractCluster(PointCloudT inputCloud, PointCloudT outputCloud)
{
//  pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
  std::vector<PointT> neighbor_it;
  for(std::size_t i=0; i < inputCloud->points.size(); ++i)
  {
    if(extractDENSITY(inputCloud->points[i]) == true)
    {
      neighbor_it.push_back(inputCloud->points[i]);
      current = inputCloud->points[i];
      break;
    }
  }
  int i=0;
  int count=0;
  while(count < 5)
  {
    if(extractDENSITY(neighbor_it[i])==true)
    {
      searchAdjacent(neighbor_it[i], neighbor_it);
      for(int num=0; num <= neighbor_it.size(); ++num)
      {
        std::cout << "Neibors == " << neighbor_it.size() << " == " << neighbor_it[num] << std::endl;
      }
      outputCloud->points.push_back(neighbor_it[i]);
      count = 0;
    } else {
//      cloud_out->points.push_back(neighbor_it[i]);
//      neighbor_it.erase(neighbor_it.begin()+i);
      count++;
    }
    if(outputCloud->points.size() == max_cluster) break;
//    std::cout << i << "== neighbor_it ==" << neighbor_it[i] << std::endl;
    i++;
  }
  std::cout << "size of cloud_out === " << outputCloud->points.size() << std::endl;

///// extract cloud_out from cloud_in. update cloud_in 
/*  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;
  for(int l=0; l < outputCloud->points.size(); ++l)
  {
    pcl::PointXYZ pt(outputCloud->points[l].x,outputCloud->points[l].y,outputCloud->points[l].z);
  }
  extract.setInputCloud(inputCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filterDirectly(inputCloud);*/

  if(outputCloud->points.size() > min_cluster) return true;
  else return false;
}
/**********************************************************************************************************/
void VoxelDBSCAN::searchAdjacent(PointT pt, std::vector<PointT> it)
{
  PointT neighbor ;
  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor); 

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
 ////////////////////////////////////////
  neighbor.x = pt.x - resolution;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x - resolution;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x - resolution;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x - resolution;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  ///////////////////////////////////////
  neighbor.x = pt.x ;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  ///////////////////////////////////////
  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y ;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y ;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y ;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y ;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  //////////////////////////////////////
  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y - resolution;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  /////////////////////////////////////
  neighbor.x = pt.x ;
  neighbor.y = pt.y ;
  neighbor.z = pt.z + resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y ;
  neighbor.z = pt.z - resolution;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  //////////////////////////////////////
  neighbor.x = pt.x ;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y + resolution;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  ///////////////////////////////////////
  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y ;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + resolution;
  neighbor.y = pt.y ;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
}
/**********************************************************************************************************/
/*void viewerOneOff(PointCloudT displayCloud )
{
  ////viewer /////////////////////////////////////////////////////////////
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("cluster viewer"));
  viewer->setBackgroundColor(0,0,0);
  viewer->addPointCloud<PointT>(displayCloud,"cluster_view");
  viewer->initCameraParameters();
  while (!viewer->wasStopped ())
  {
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  viewer->removePointCloud("displayCloud");
}*/
/**********************************************************************************************************/

int main(int argc, char const *argv[])
{
  /* code */
  std::string pcd_path(argv[1]);
  VoxelDBSCAN dbscan(pcd_path, atof(argv[2]));
  return 0;
}