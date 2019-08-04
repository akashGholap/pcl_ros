#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
void storefilename_callback(const std_msgs::String& pcd_file_name);
std::ofstream pcdfile_write;
int hopped = 0;
int counter=0;

struct PCD
{
  PointCloudT::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloudT) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

// Align a rigid object to a scene with clutter and occlusions
void loadData_from_txt (std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  int argc =0;
  std::string line;
  ROS_INFO("In LoadData Loop");
  // Suppose the first argument is the actual test model
  ifstream pcdfile_linecount ("pcd_file_list.txt");
  if (pcdfile_linecount.is_open())
  {
    ROS_INFO("counting the arguments");
    while(getline (pcdfile_linecount,line))
    {

      argc++;
    }
    ROS_INFO("%d",argc);
    pcdfile_linecount.close();
  }
  else
  {
    ROS_INFO("unable to open the file");
  }
  ROS_INFO("In LoadData Loop");
  ifstream pcdfile_read ("pcd_file_list.txt");


  for (int i = 1; i < argc; i++)
  {
    std::stringstream ss;
    if (pcdfile_read.is_open())
    {
    getline (pcdfile_read,line) ;
    ss << line;
    ROS_INFO("Loading Data %s", ss.str().c_str());

    }
    std::string fname = ss.str();
    // Needs to be at least 5: .plot
    ROS_INFO("fname %s", fname.c_str());
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    ROS_INFO("Is the compare output %d",fname.compare (fname.size () - extension.size (), extension.size (), extension) );
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      ROS_INFO("Successfully Matched");
      PCD m;
      m.f_name = ss.str();
      pcl::io::loadPCDFile<PointNT> (ss.str(), *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
  pcdfile_read.close();
}


int main (int argc, char **argv)
{
  // Point clouds
  ros::init(argc,argv,"icp_ros_runtime");
	ros::NodeHandle nh;
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  ros::Subscriber pcdFileListSub = nh.subscribe("/pcd_file_string", 10, &storefilename_callback);
  pcdfile_write.open ("pcd_file_list.txt");
  loadData_from_txt(data);
  while(ros::ok())
  {
    if(hopped)
    {

      Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
      for(size_t i = 1; i < data.size (); ++i)
      {
      PointCloudT::Ptr object (new PointCloudT);
      PointCloudT::Ptr object_aligned (new PointCloudT);
      PointCloudT::Ptr scene (new PointCloudT);
      FeatureCloudT::Ptr object_features (new FeatureCloudT);
      FeatureCloudT::Ptr scene_features (new FeatureCloudT);

      object = data[i-1].cloud;
      scene = data[i].cloud;
      // Get input object and scene
      // Load object and scene
      // Downsample
      pcl::console::print_highlight ("Downsampling...\n");
      pcl::VoxelGrid<PointNT> grid;
      const float leaf = 0.005f;
      grid.setLeafSize (leaf, leaf, leaf);
      grid.setInputCloud (object);
      grid.filter (*object);
      grid.setInputCloud (scene);
      grid.filter (*scene);

      // Estimate normals for scene
      pcl::console::print_highlight ("Estimating scene normals...\n");
      pcl::NormalEstimationOMP<PointNT,PointNT> nest, zest;
      nest.setRadiusSearch (0.01);
      nest.setInputCloud (scene);
      nest.compute (*scene);
      zest.setRadiusSearch (0.01);
      zest.setInputCloud (object);
      zest.compute (*object);

      // Estimate features
      pcl::console::print_highlight ("Estimating features...\n");
      FeatureEstimationT fest;
      fest.setRadiusSearch (0.025);
      fest.setInputCloud (object);
      fest.setInputNormals (object);
      fest.compute (*object_features);
      fest.setInputCloud (scene);
      fest.setInputNormals (scene);
      fest.compute (*scene_features);

      // Perform alignment
      pcl::console::print_highlight ("Starting alignment...\n");
      pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
      align.setInputSource (object);
      align.setSourceFeatures (object_features);
      align.setInputTarget (scene);
      align.setTargetFeatures (scene_features);
      align.setMaximumIterations (50000); // Number of RANSAC iterations
      align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
      align.setCorrespondenceRandomness (5); // Number of nearest features to use
      align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
      align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
      align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
      {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
      }

      if (align.hasConverged ())
      {
        // Print results
        printf ("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        // Show alignment
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();
      }
      else
      {
        pcl::console::print_error ("Alignment failed!\n");
      }
      GlobalTransform *= align.getFinalTransformation();
      cout<<GlobalTransform<<endl;
    }

    }
  }

}

void storefilename_callback(const std_msgs::String& pcd_file_name)
{
  if(hopped == 0)
  {
  //ROS_INFO("SphereX Hopping");
	std::stringstream ss;
	ss << pcd_file_name.data << ".pcd";
  ROS_INFO("%s",ss.str().c_str());

//myfile << "This is the first cell in the first column.
	pcdfile_write << ss.str()<<endl;
  //pcdfile_write << " Ye errorwa nahi samazh aa raha";
	//pcdfile_write.close();
  counter++;

  }
  else ROS_INFO("SphereX hopped %d", hopped);
  if(counter >= 10)
  {
    hopped=1;
  }
  else hopped = 0;
}
