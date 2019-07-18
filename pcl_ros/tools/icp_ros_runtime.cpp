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

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
class SphereX
{
  public:
	bool hop_status;
	bool icp_status;

	SphereX() : hop_status(false),icp_status(false){}

};//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
PointCloud::Ptr global_cloud (new PointCloud), cloud_inRadius (new PointCloud);
std::ofstream pcdfile_write;
int counter=0;
SphereX hop;
//std::ofstream pcdfile_read;
void storefilename_callback(const std_msgs::String& pcd_file_name);
// This is a tutorial so we can afford having global variables
	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
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


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  //PCL_INFO ("Press q to begin the registration.\n");
  //p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
//  myfile_read
  //ROS_INFO("In LoadData Loop");
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}
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
      pcl::io::loadPCDFile (ss.str(), *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
  pcdfile_read.close();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (10);
  for (int i = 0; i < 10; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
  //p->spin ();

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }


/* ---[ */
int main (int argc, char** argv)
{
  // Load data
  //ROS_INFO("main");
  ros::init(argc,argv,"icp_ros_runtime");
	ros::NodeHandle nh;
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  ros::Subscriber pcdFileListSub = nh.subscribe("/pcd_file_string", 10, &storefilename_callback);
  pcdfile_write.open ("pcd_file_list.txt");
  while(ros::ok())
  {
      //ROS_INFO("%d", hopped);

      if(hop.hop_status==true)
      {
          ROS_INFO("in Hopped");
          //loadData (argc, argv, data);
          loadData_from_txt(data);
        // Check user input
          ROS_INFO("Missed Load data");
          if (data.empty ())
          {
            PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
            PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
            return (-1);
          }
          PCL_INFO ("Loaded %d datasets.", (int)data.size ());

          // Create a PCLVisualizer object
          p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
          p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
          p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

        	PointCloud::Ptr result (new PointCloud), source, target;
          Eigen::Matrix4f /*GlobalTransform = Eigen::Matrix4f::Identity (),*/ pairTransform;

          for (size_t i = 1; i < data.size (); ++i)
          {
            source = data[i-1].cloud;
            target = data[i].cloud;

            // Add visualization data
            //showCloudsLeft(source, target);

            PointCloud::Ptr temp (new PointCloud);
            PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i].f_name.c_str (), source->points.size (), data[i-1].f_name.c_str (), target->points.size ());
            pairAlign (source, target, temp, pairTransform, true);

            GlobalTransform *= pairTransform;


            pcl::transformPointCloud(*target, *result, GlobalTransform);


            *global_cloud += *result;
            //transform current pair into the global transform
            //pcl::transformPointCloud (*temp, *result, GlobalTransform);

            //update the global transform

            std::cout << GlobalTransform << std::endl;

            std::cout << "Now lets go to the local pairtransform" << std::endl;

            std::cout << pairTransform << std::endl;



            //save aligned pair, transformed into the first cloud's frame


         }
           std::stringstream ss;
           ss << "1.pcd";
           pcl::io::savePCDFile (ss.str (), *global_cloud, true);
           counter = 0;
					 hop.icp_status = true;



       }
			 if(hop.hop_status && hop.icp_status)
			 {
					 double x_ = GlobalTransform(0, 3);
					 double y_ = GlobalTransform(1, 3);
					 double z_ = GlobalTransform(2, 3);
					 double r = 3;

					 int number_of_indices = 70000 ;
					 std::vector<float> k_radius(number_of_indices);
					 std::vector<int> k_indices(number_of_indices);
					 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		       kdtree.setInputCloud (global_cloud);
					 pcl::PointXYZ searchPoint;
					 searchPoint.x = x_;
					 searchPoint.y = y_;
					 searchPoint.z = z_;
					 pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
					 kdtree.radiusSearch(searchPoint, r , k_indices, k_radius);
					 inliers->indices = k_indices;
					 pcl::ExtractIndices<pcl::PointXYZ> filter_pc ; // Initializing with true will allow us to extract the removed indices
           filter_pc.setInputCloud(global_cloud);
           filter_pc.setIndices(inliers);
           filter_pc.filter(*cloud_inRadius);
					 std::stringstream ss;
           ss << "2.pcd";
           pcl::io::savePCDFile (ss.str (), *cloud_inRadius, true);
					 hop.icp_status = false;
		   }


       ros::spinOnce();
   }



}
void storefilename_callback(const std_msgs::String& pcd_file_name)
{
  if(!hop.hop_status && !hop.icp_status)
  {
  //ROS_INFO("SphereX Hopping");
  if(counter % 2 == 0){
	std::stringstream ss;
	ss << pcd_file_name.data << ".pcd";
  ROS_INFO("%s",ss.str().c_str());

//myfile << "This is the first cell in the first column.
	pcdfile_write << ss.str()<<endl;
  //pcdfile_write << " Ye errorwa nahi samazh aa raha";
	//pcdfile_write.close();
  }
  counter++;

  }
  else ROS_INFO("SphereX hopped");
  if(counter >= 60)
  {
    hop.hop_status=true;
  }
  else hop.hop_status = false;
}
