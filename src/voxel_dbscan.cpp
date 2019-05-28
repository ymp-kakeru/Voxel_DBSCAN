#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_impl.h> //octreeの各種テンプレートの使用にはここのマクロ？が必要らしい。Densityだけっぽいけど
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
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
  VoxelDBSCAN();
  bool loadPCD(std::string filename);
  void GenerateOctree();
  void dbscan();
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *);
  void run();
  void showLegend(bool showCubes);
  void update();
  void clearView();
  void showCubes(double voxelSideLen);
  void extractPointsAtLevel(int depth);
  bool IncrementLevel();
  bool DecrementLevel();

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;
  pcl::visualization::PCLVisualizer viz;

  PointCloudT cloud_in;
  PointCloudT displayCloud ;
  pcl::octree::OctreePointCloudDensity<PointT> octree;

  std::string FileName;
  const double resolution_tree ;
  int displayedDepth ;
  bool displayCubes, showPointsWithCubes, wireframe;

};
/*****************************************************************************************************/
VoxelDBSCAN::VoxelDBSCAN():
  resolution_tree{10.0f},FileName{"/home/ymp/catkin_ws/src/pcl_something/hill_map.pcd.pcd"},
  octree(resolution_tree),
  viz("Octree visualizer"), displayCubes(false),showPointsWithCubes(false),wireframe(true)
{
  loadPCD(FileName);
  GenerateOctree();
  //register keyboard callbacks
  viz.registerKeyboardCallback(&OctreeViewer::keyboardEventOccurred, *this, 0);

  //key legends
  viz.addText("Keys:", 0, 170, 0.0, 1.0, 0.0, "keys_t");
  viz.addText("a -> Increment displayed depth", 10, 155, 0.0, 1.0, 0.0, "key_a_t");
  viz.addText("z -> Decrement displayed depth", 10, 140, 0.0, 1.0, 0.0, "key_z_t");
  viz.addText("d -> Toggle Point/Cube representation", 10, 125, 0.0, 1.0, 0.0, "key_d_t");
  viz.addText("x -> Show/Hide original cloud", 10, 110, 0.0, 1.0, 0.0, "key_x_t");
  viz.addText("s/w -> Surface/Wireframe representation", 10, 95, 0.0, 1.0, 0.0, "key_sw_t");

  //set current level to half the maximum one
  displayedDepth = static_cast<int> (floor (octree.getTreeDepth() / 2.0));
  if (displayedDepth == 0)
    displayedDepth = 1;

  //show octree at default depth
  extractPointsAtLevel(displayedDepth);

  //reset camera
  viz.resetCameraViewpoint("cloud");

  //run main loop
  run();

}

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
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

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

  /* Get Leaf Voxel Density */
  while(*++leaf_it)
  {
//    std::cout << leaf_it.operator*() << std::endl;
//    std::cout << octree.getVoxelDensityAtPoint(leaf_it.operator*()) << std::endl ;
//    std::cout << octree.getVoxelDensityAtPoint(PointT(1,1,1)) << std::endl ;
  }

/*  for(pcl::octree::OctreePointCloud<PointT>:: node=octree.leaf_begin(); node!=octree.leaf_end(); ++node)
  {
     std::cout << octree.getVoxelDensityAtPoint(it) << std::endl ;
  }*/

//  std::cout << octree.getResolution() << std::endl ;
}
/**********************************************************************************************************/
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
{

  if (event.getKeySym() == "a" && event.keyDown())
  {
    IncrementLevel();
  }
  else if (event.getKeySym() == "z" && event.keyDown())
  {
    DecrementLevel();
  }
  else if (event.getKeySym() == "d" && event.keyDown())
  {
    displayCubes = !displayCubes;
    update();
  }
  else if (event.getKeySym() == "x" && event.keyDown())
  {
    showPointsWithCubes = !showPointsWithCubes;
    update();
  }
  else if (event.getKeySym() == "w" && event.keyDown())
  {
    if(!wireframe)
      wireframe=true;
    update();
  }
  else if (event.getKeySym() == "s" && event.keyDown())
  {
    if(wireframe)
      wireframe=false;
    update();
  }
}
/**********************************************************************************************************/
void run()
{
  while (!viz.wasStopped())
  {
    //main loop of the visualizer
    viz.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
/**********************************************************************************************************/
void showLegend(bool showCubes)
{
  char dataDisplay[256];
  sprintf(dataDisplay, "Displaying data as %s", (showCubes) ? ("CUBES") : ("POINTS"));
  viz.removeShape("disp_t");
  viz.addText(dataDisplay, 0, 60, 1.0, 0.0, 0.0, "disp_t");

  char level[256];
  sprintf(level, "Displayed depth is %d on %d", displayedDepth, octree.getTreeDepth());
  viz.removeShape("level_t1");
  viz.addText(level, 0, 45, 1.0, 0.0, 0.0, "level_t1");

  viz.removeShape("level_t2");
  sprintf(level, "Voxel size: %.4fm [%zu voxels]", sqrt(octree.getVoxelSquaredSideLen(displayedDepth)),
          displayCloud->points.size());
  viz.addText(level, 0, 30, 1.0, 0.0, 0.0, "level_t2");

  viz.removeShape("org_t");
  if (showPointsWithCubes)
    viz.addText("Displaying original cloud", 0, 15, 1.0, 0.0, 0.0, "org_t");
}
/**********************************************************************************************************/
void update()
{
  //remove existing shapes from visualizer
  clearView();

  //prevent the display of too many cubes
  bool displayCubeLegend = displayCubes && static_cast<int> (displayCloud->points.size ()) <= MAX_DISPLAYED_CUBES;

  showLegend(displayCubeLegend);

  if (displayCubeLegend)
  {
    //show octree as cubes
    showCubes(sqrt(octree.getVoxelSquaredSideLen(displayedDepth)));
    if (showPointsWithCubes)
    {
      //add original cloud in visualizer
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
      viz.addPointCloud(cloud, color_handler, "cloud");
    }
  }
  else
  {
    //add current cloud in visualizer
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(displayCloud,"z");
    viz.addPointCloud(displayCloud, color_handler, "cloud");
  }
}
/**********************************************************************************************************/
void clearView()
{
  //remove cubes if any
  vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();
  while (renderer->GetActors()->GetNumberOfItems() > 0)
    renderer->RemoveActor(renderer->GetActors()->GetLastActor());
  //remove point clouds if any
  viz.removePointCloud("cloud");
}

/* \brief Create a vtkSmartPointer object containing a cube
 *
 */
vtkSmartPointer<vtkPolyData> GetCuboid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
  cube->SetBounds(minX, maxX, minY, maxY, minZ, maxZ);
  return cube->GetOutput();
}
/**********************************************************************************************************/
void showCubes(double voxelSideLen)
{
  //get the renderer of the visualizer object
  vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();

  vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New();
  size_t i;
  double s = voxelSideLen / 2.0;
  for (i = 0; i < displayCloud->points.size(); i++)
  {

    double x = displayCloud->points[i].x;
    double y = displayCloud->points[i].y;
    double z = displayCloud->points[i].z;

    treeWireframe->AddInput(GetCuboid(x - s, x + s, y - s, y + s, z - s, z + s));
  }

  vtkSmartPointer<vtkActor> treeActor = vtkSmartPointer<vtkActor>::New();

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
  mapper->SetInput(treeWireframe->GetOutput());
  treeActor->SetMapper(mapper);

  treeActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  treeActor->GetProperty()->SetLineWidth(2);
  if(wireframe)
  {
    treeActor->GetProperty()->SetRepresentationToWireframe();
    treeActor->GetProperty()->SetOpacity(0.35);
  }
  else
    treeActor->GetProperty()->SetRepresentationToSurface();

  renderer->AddActor(treeActor);
}
/**********************************************************************************************************/
void extractPointsAtLevel(int depth)
{
  displayCloud->points.clear();

  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it_end = octree.end();

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
    displayCloud->points.push_back(pt);
  }

  double end = pcl::getTime ();
  printf("%zu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
         (end - start) / static_cast<double> (displayCloud->points.size ()));

  update();
}
/**********************************************************************************************************/
bool IncrementLevel()
{
  if (displayedDepth < static_cast<int> (octree.getTreeDepth ()))
  {
    displayedDepth++;
    extractPointsAtLevel(displayedDepth);
    return true;
  }
  else
    return false;
}
/**********************************************************************************************************/
bool DecrementLevel()
{
  if (displayedDepth > 0)
  {
    displayedDepth--;
    extractPointsAtLevel(displayedDepth);
    return true;
  }
  return false;
}
/**********************************************************************************************************/
void VoxelDBSCAN::dbscan()
{

}


int main(int argc, char const *argv[])
{
  /* code */
  VoxelDBSCAN dbscan;
  return 0;
}