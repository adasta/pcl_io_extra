/*
 * pcl_e572pcd.cpp
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */


#include <pcl/io/pcd_io.h>
#include <pcl/io/e57_io.h>

#include <iostream>

int main(int argc, char** argv){

  if (argc!=3){
    std::cout << "./pcl_572pcd <e57>  <pcd>\n";
  }

  pcl::E57Reader reader;

  sensor_msgs::PointCloud2 cloud;

  Eigen::Vector4f origin;
  Eigen::Quaternionf rot;

  int fv;
  int c = reader.read(argv[1],cloud, origin, rot, fv);
  std::cout << "There were " << c << " points \n";

  pcl::PCDWriter writer;

  writer.write(argv[2], cloud,origin, rot);
}
