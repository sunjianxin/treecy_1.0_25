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
#include <fstream>
#include <string>

typedef pcl::PointXYZ PointT;

int
main (int argc, char *argv[])
{
  // float r; //resolution
  // r = atof(argv[1]);
  // All the objects needed
  pcl::PCDReader reader;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_1;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_2;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_1;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_2; 
  pcl::PCDWriter writer;
  pcl::search::KdTree<PointT>::Ptr tree_1 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_2 (new pcl::search::KdTree<PointT> ());
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_temp_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_temp_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_1 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_1 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_2 (new pcl::PointIndices);


  pcl::PointXYZ basic_point;

  // Read in the cloud data
  reader.read ("../../tree25/tree_longbranch_scaled.pcd", *cloud);
  std::cerr << "Input PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_temp);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud 1");
  //viewer->addPointCloud<pcl::PointXYZ> (cloud_temp_2, "sample cloud 2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud 1");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud 2");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  std::cout << "hiiii" << std::endl;

  std::string line;
  for (int j = 0; j < 3; ++j){
    std::stringstream ss;
    ss << "../../tree25/section0-0.36_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 3; ++j){
    std::stringstream ss;
    ss << "../../tree25/section0.36-0.66_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 1; ++j){
    std::stringstream ss;
    ss << "../../tree25/section0.66-1.17_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 3; ++j){
    std::stringstream ss;
    ss << "../../tree25/section1.17-1.32_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 1; ++j){
    std::stringstream ss;
    ss << "../../tree25/section1.32-1.71_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 1; ++j){
    std::stringstream ss;
    ss << "../../tree25/section1.71-1.89_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 1; ++j){
    std::stringstream ss;
    ss << "../../tree25/section1.89-2.01_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 1; ++j){
    std::stringstream ss;
    ss << "../../tree25/section2.01-2.22_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 2; ++j){
    std::stringstream ss;
    ss << "../../tree25/section2.22-2.43_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 2; ++j){
    std::stringstream ss;
    ss << "../../tree25/section2.43-2.49_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 3; ++j){
    std::stringstream ss;
    ss << "../../tree25/section2.49-2.76_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}

    for (int j = 0; j < 2; ++j){
    std::stringstream ss;
    ss << "../../tree25/section2.76-2.85_branch-" << j << ".txt";
    std::cout << ss.str() << std::endl;
    std::ifstream myfile(ss.str().c_str());
    if (myfile.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, ss.str());
    coefficients_cylinder_1->values.clear();
    myfile.close();}


  /*
  //for (int j = 0; j < 9; ++j){
    std::string line;
    //std::ifstream myfile;
    std::ifstream myfile0("../../locateTrunk/build/cutted_tree_0-0.25.txt");
    // myfile.open("../../locateTrunk/build/cutted_tree_0-0.25.txt");
    if (myfile0.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile0, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, "cylinder_1");
    myfile0.close();
    
    coefficients_cylinder_1->values.clear();
    //myfile("../../locateTrunk/build/cutted_tree_0.25-0.52.txt");
    std::ifstream myfile1("../../locateTrunk/build/cutted_tree_0.25-0.52.txt");
    if (myfile1.is_open()){
      for (int i = 0; i < 7; ++i){
	getline(myfile1, line);
	std::cout << line << "\n";
	coefficients_cylinder_1->values.push_back(atof(line.c_str()));}}
    viewer->addCylinder(*coefficients_cylinder_1, "cylinder_2");
    myfile1.close();  
  */










  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
