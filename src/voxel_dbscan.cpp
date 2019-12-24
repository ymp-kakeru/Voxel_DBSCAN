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
  void extractCluster(PointCloudT inputCloud);
  void searchAdjacent(PointT pt,std::vector<PointT> it);
  void run();

  /*pcl viewer functions*/
//  void viewerOneOff(PointCloudT displayCloud);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;

  PointCloudT cloud_in;
  PointCloudT octree_centroid_pts;
  PointT current;
  pcl::octree::OctreePointCloudDensity<PointT> octree;

  double minpts ,resolution;
  int min_cluster, max_cluster;
  Eigen::Vector3f voxel_min, voxel_max;

};
/*****************************************************************************************************/
VoxelDBSCAN::VoxelDBSCAN(std::string & FileName, double resolution_tree):
  octree(resolution_tree),cloud_in (new pcl::PointCloud<pcl::PointXYZ>()),minpts{0},resolution{resolution_tree},
  min_cluster{10},max_cluster{5000}
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
  extractCluster(octree_centroid_pts);
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
void VoxelDBSCAN::extractCluster(PointCloudT inputCloud)
{
  pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
//  pcl::octree::OctreePointCloudDensity<PointT>::Iterator tree_it;
  std::vector<PointT> neighbor_it;
 /* for(tree_it = octree.leaf_begin(); tree_it != octree.leaf_end(); ++tree_it)
  {
    octree.getVoxelBounds(tree_it,voxel_min,voxel_max);
    current.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
    current.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
    current.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
//    std::cout <<  "get voxel density : "<< octree.getVoxelDensityAtPoint(current) << std::endl;
    if(extractDENSITY(current) == true)
    {
      neighbor_it.push_back(current);
      break;
    }
  }*/
  int i=0;
  int count=0;
  while(count < 20)
  {
    if(extractDENSITY(neighbor_it[i])==true){
      searchAdjacent(neighbor_it[i], neighbor_it);
      cloud_out->points.push_back(neighbor_it[i]);
      count = 0;
    } else {
//      cloud_out->points.push_back(neighbor_it[i]);
//      neighbor_it.erase(neighbor_it.begin()+i);
      count++;
    }
    if(cloud_out->points.size() == max_cluster) break;
    i++;
    std::cout << cloud_out->points[i] << std::endl;
  }
  std::cout << "size of cloud_out === " << cloud_out->points.size() << std::endl;

///// extract cloud_out from cloud_in. update cloud_in 
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;
  for(int l=0; l < cloud_out->points.size(); ++l)
  {
    pcl::PointXYZ pt(cloud_out->points[l].x,cloud_out->points[l].y,cloud_out->points[l].z);
  }
  extract.setInputCloud(inputCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filterDirectly(inputCloud);

/*viewer /////////////////////////////////////////////////////////////*/
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
void VoxelDBSCAN::searchAdjacent(PointT pt, std::vector<PointT> it)
{
  PointT neighbor ;
  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor); 

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
 ////////////////////////////////////////
  neighbor.x = pt.x - (pt.x / 2.0f);
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x - (pt.x / 2.0f);
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x - (pt.x / 2.0f);
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x - (pt.x / 2.0f);
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  ///////////////////////////////////////
  neighbor.x = pt.x ;
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  ///////////////////////////////////////
  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y ;
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y ;
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y ;
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y ;
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  //////////////////////////////////////
  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y - (pt.y / 2.0f);
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  /////////////////////////////////////
  neighbor.x = pt.x ;
  neighbor.y = pt.y ;
  neighbor.z = pt.z + (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y ;
  neighbor.z = pt.z - (pt.z / 2.0f);
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  //////////////////////////////////////
  neighbor.x = pt.x ;
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x ;
  neighbor.y = pt.y + (pt.y / 2.0f);
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
  ///////////////////////////////////////
  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y ;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);

  neighbor.x = pt.x + (pt.x / 2.0f);
  neighbor.y = pt.y ;
  neighbor.z = pt.z ;
  if((pcl::geometry::distance(current,neighbor)+sqrt(resolution)) > (pcl::geometry::distance(current,pt)))
  it.push_back(neighbor);
}
/**********************************************************************************************************/


/**********************************************************************************************************/

int main(int argc, char const *argv[])
{
  /* code */
  std::string pcd_path(argv[1]);
  VoxelDBSCAN dbscan(pcd_path, atof(argv[2]));
  return 0;
}