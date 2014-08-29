#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <cmath>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <fstream>
#include <vector>
//#indlude <>

using namespace std;

typedef pcl::PointXYZ PointT;

int
main (int argc, char *argv[])
{
  float DistanceThreshold; //circle model distance threshold
  DistanceThreshold = atof(argv[1]);
  float radius_lower; //circle model radius lower bound
  radius_lower = atof(argv[2]);
  float radius_upper; //circle model radius lower bound
  radius_upper = atof(argv[3]);
  // All the objects needed
  pcl::PCDReader reader;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_1;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_2;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_1;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_2; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree_1 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_2 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_3 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_4 (new pcl::search::KdTree<PointT> ());
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_1_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_4 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr layer_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_temp_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_temp_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_3 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_4 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_1 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_1 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_2 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_3 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_3 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_4 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_4 (new pcl::PointIndices);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr layer_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointT>::Ptr inlier_cloud_1 (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr inlier_cloud_2 (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr inlier_cloud_3 (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr inlier_cloud_4 (new pcl::PointCloud<PointT> ());

  pcl::PointXYZ basic_point;
  pcl::PointXYZRGB point;
  // Read in the cloud data
  reader.read ("tree_scaled.pcd", *cloud);
  std::cerr << "Input PointCloud has: " << cloud->points.size () << " data points." << std::endl;


  float thickness = 0.03; //layer thickness //0.05
  int layer_number;
  float tree_height = cloud->points[1].z;
  
  for (int i = 0; i < cloud->points.size(); ++i){
    if (cloud->points[i].z > tree_height){
      tree_height = cloud->points[i].z;}}
  layer_number = int(ceil(tree_height/thickness));
  int diameter[layer_number];
  int count;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 viewer->setBackgroundColor (0, 0, 0);
 //viewer->addCoordinateSystem (1.0);
 viewer->initCameraParameters ();



 std::stringstream sss;
 sss << "outlierNumberAt" << thickness << "thickness" << ".txt";
 //std::cout << sss.str() << std::endl;
 std::ofstream myfile;
 myfile.open(sss.str().c_str());
 //for (int i = 0; i < 7; ++i){
 //  myfile << coefficients_cylinder_2->values[i] << "\n";}
 //myfile << cylength << "\n";
 //myfile.close();


 //------circle model on projection of each layer
 for (int i = 0; i < layer_number; ++i){
   //construct the layer cloud for "i"th layer
    count = 0;
    // int i = 30;
    pcl::PointCloud<PointT>::Ptr layer (new pcl::PointCloud<PointT>);
    for (int j = 0; j < cloud->points.size(); ++j){
      if (cloud->points[j].z > i*thickness  && cloud->points[j].z < (i+1)*thickness){
	count += 1;
	layer->points.push_back(cloud->points[j]);}}
    layer->width = count;
    layer->height = 1;
    layer->points.resize(count);
    for (int m = 0; m < layer->points.size(); ++m){
      layer->points[m].z = i*thickness;} //i*thickness
   
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr layer_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r(255), g(15), b(15);
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	    
	    
      
  
    //-----------start circle modeling
    
    //----------------------------------------------process 1---------------------
    //RANSAC
    ne_1.setSearchMethod (tree_1);
    ne_1.setInputCloud (layer); 
    ne_1.setKSearch (50);
    ne_1.compute (*cloud_normals_1);

  

    //Create the segmentation object for cylinder segmentation and set all the parameters
    seg_1.setOptimizeCoefficients (true);
    seg_1.setModelType (pcl::SACMODEL_CIRCLE2D); //CYLINDER
    seg_1.setMethodType (pcl::SAC_RANSAC);
    seg_1.setNormalDistanceWeight (0.1);
    seg_1.setMaxIterations (10000);
    seg_1.setDistanceThreshold (DistanceThreshold); //0.02 is used on sythentic tree; 0.05 used in dave's cut then registered trunk//0.01 better than original 0.05
    seg_1.setRadiusLimits (radius_lower, radius_upper);
    //seg_1.setRadiusLimits (0.05, 0.06); 
    seg_1.setInputCloud (layer);
    seg_1.setInputNormals (cloud_normals_1);

    seg_1.segment (*inliers_cylinder_1, *coefficients_cylinder_1);
    //----------------------------------------------process 1---------------------
    // find inlier in this layer
    extract.setInputCloud (layer);
    extract.setIndices (inliers_cylinder_1);
    extract.setNegative (false);
    extract.filter (*inlier_cloud_1);
 

    extract.setNegative (true);
    extract.filter (*rest_cloud_1);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals_1);
    extract_normals.setIndices (inliers_cylinder_1);
    extract_normals.filter (*cloud_normals_2);

    //writer.write ("rest_cloud.pcd", *rest_cloud_1, false);


    

    std::cout << "outliers number: " << rest_cloud_1->points.size() << std::endl;


    std::cerr << "Cylinder 1 coefficients: " << *coefficients_cylinder_1 << std::endl;
  
    //     count = 0;  
    for (int m = 0; m < rest_cloud_1->points.size(); ++m){
      point.x = rest_cloud_1->points[m].x;
      point.y = rest_cloud_1->points[m].y;
      point.z = rest_cloud_1->points[m].z;
      point.rgb = *reinterpret_cast<float*>(&rgb);
      layer_cloud_rgb->points.push_back(point);
      //count += 1;
    }
    layer_cloud_rgb->width = rest_cloud_1->points.size();
    layer_cloud_rgb->height = 1;
    layer_cloud_rgb->points.resize(rest_cloud_1->points.size());
  
    std::stringstream sslayer;
    sslayer << "layer" << "-" << i;
    std::stringstream ssoutlier;
    ssoutlier << "outlier" << "-" << i;
    std::stringstream sscircle;
    sscircle << "circle" << "-" << i;


    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

 

    //color cloud plot

    viewer->addPointCloud<pcl::PointXYZ> (layer, sslayer.str());
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbfield(layer_cloud_rgb);
    viewer->addPointCloud<pcl::PointXYZRGB> (layer_cloud_rgb, rgbfield, ssoutlier.str());
    //viewer->addPointCloud<pcl::PointXYZ> (layer, "sample cloud 1");
    //viewer->addPointCloud<pcl::PointXYZ> (rest_cloud_1, "sample cloud 2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, sslayer.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ssoutlier.str());
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud 2");
 
    //return (viewer);


    //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (cloud_temp);
    viewer->addCircle(*coefficients_cylinder_1, sscircle.str());
    std::cerr << "Circle coefficients: " << *coefficients_cylinder_1 << std::endl;

    myfile << rest_cloud_1->points.size() << "\n";



 

    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (layer_cloud, cloud_normals_1, 10, 0.05, "normals");



 }

 myfile.close();

 //Calculate Cutting Point
 std::string line;
 std::ifstream rmyfile("outlierNumberAt0.03thickness.txt");
 std::vector<float> array;
 std::vector<float> left;
 std::vector<float> right;
 std::vector<float> cutpoint;
 int index = 0;
 if (rmyfile.is_open()){
   //for (int i = 0; i < 7; ++i){
   
   while (getline(rmyfile, line)){
     array.push_back(atof(line.c_str()));
     index += 1;}}
 rmyfile.close();
 for (int i = 0; i < index; ++i){
   
   if ((array.at(i) == 0 && i == 0) || (array.at(i) == 0 && i != 0 && array.at(i-1) != 0)){
     left.push_back(i);}
   if ((array.at(i) == 0 && i == array.size()-1) || (array.at(i) == 0 && i != array.size()-1 && array.at(i+1) != 0)){
     right.push_back(i);}}
   //std::cout << "array" << array.at(i) << std::endl;}
 for (int i = 0; i < left.size(); ++i){
   std::cout << "left: " << left.at(i) << std::endl;}
 for (int i = 0; i < right.size(); ++i){
   std::cout << "right: " << right.at(i) << std::endl;}

 for (int i = 0; i < left.size(); ++i){
   std::cout << "interval " << i << ": " << (right.at(i)-left.at(i))*0.03 << std::endl;}

 for (int i = 0; i < left.size()-1; ++i){
   std::cout << "clear trunk: " << left.at(i)*0.03 << "--" << right.at(i)*0.03 << std::endl;
   std::cout << "branch trunk: " << right.at(i)*0.03 << "--" << left.at(i+1)*0.03 << std::endl;} 


 for (int i = 0; i < left.size(); ++i){
   cutpoint.push_back(right.at(i)*0.03);}
 // cutpoint.insert(cutpoint.begin(), 0.25);

 //v1.begin()+i

 for (int i = 0; i < cutpoint.size(); ++i){
   std::cout << "Cutting Points: " << cutpoint.at(i) << std::endl;}


 //std::stringstream sss;
 //sss << "cutted_tree_" << res_lower << "-" <<res_upper << ".txt";
 //std::cout << sss.str() << std::endl;
  std::ofstream cutpoint_myfile;
  cutpoint_myfile.open("cutpoint.txt");
  for (int i = 0; i < cutpoint.size(); ++i){
    cutpoint_myfile << cutpoint.at(i) << "\n";}
  cutpoint_myfile.close();

  std::ofstream possiblecutpoint_myfile;
  possiblecutpoint_myfile.open("possiblecutpoint.txt");
  for (int i = 0; i < left.size(); ++i){
    possiblecutpoint_myfile << (left.at(i)-0.5)*0.03 << "\n";
    possiblecutpoint_myfile << (right.at(i)-0.5)*0.03 << "\n";
    possiblecutpoint_myfile << "\n";}
  possiblecutpoint_myfile.close();


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
