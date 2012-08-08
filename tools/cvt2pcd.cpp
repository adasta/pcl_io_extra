/*
 * pcl_e572pcd.cpp
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */


#include <pcl/io/pcd_io.h>
#include <pcl/io/cloud_io.h>
#include <pcl/io/ptx_io.h>
#include <pcl/point_types.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po=boost::program_options;

#ifdef E57
#include <pcl/io/e57_io.h>
#endif
#ifdef LAS
#include <pcl/io/las_io.h>
#endif


#include <iostream>
#include <vector>

int main(int argc, char** argv){

  po::options_description desc("./cvt2pcd [options] input_cloud output_pcd");

  desc.add_options()
          ("help", "produce help message")
          ("input,i",po::value<std::string>()->required(), "input point cloud ")
          ("output,o",po::value<std::string>(), "output pcd ")
          ("view_offset,v", po::value< std::vector<double> >()->multitoken(), "Offset a view point translation (HACK for dealing with floating point precision)")
          ("ascii,a", "PCD should be in asci format.  (Default binary compressed)")
          ;

  if (argc <3 ){
           std::cout << "Incorrect number of arguments\n";
           desc.print(std::cout);
           return -1;
   }
  po::positional_options_description p;
  p.add("input",1);
  p.add("output",1);

  po::variables_map vm;
 try{
  po::store(po::command_line_parser(argc, argv).
  options(desc).positional(p).run(), vm);
  po::notify(vm);
 }
 catch( const std::exception& e)
 {
     std::cerr << e.what() << std::endl;
     std::cout << desc << std::endl;
     return 1;
 }


  Eigen::Vector4d voffset;
  if (vm.count("view_offset")) {
    std::vector<double> o =vm["view_offset"].as<std::vector<double> >();
    for(int i=0; i<3; i++) voffset[i] = o[i];
    voffset[3] = 1;
  }
  pcl::CloudReader reader;

  std::string ifile, ofile;
  ifile = vm["input"].as<std::string>();
  if(vm.count("output")) ofile = vm["output"].as<std::string>();
  else {
    ofile = ifile;
    boost::filesystem::path opath = ofile;
    opath.replace_extension(".pcd");
    ofile = opath.string();
  }

#ifdef E57
  reader.registerExtension("e57", new pcl::E57Reader(voffset));
#endif
#ifdef LAS
  reader.registerExtension("las", new pcl::LASReader );
#endif
  sensor_msgs::PointCloud2 cloud;

  Eigen::Vector4f origin;
  Eigen::Quaternionf rot;

  int fv;
  std::clog << "Loading " <<  ifile << " \n";
  int c = reader.read(ifile,cloud, origin, rot, fv);
  std::clog << "There were " << c << " points \n";

  pcl::PCDWriter writer;
  writer.write(ofile, cloud,origin, rot, !vm.count("ascii"));
  std::cout << ofile << "\n";
}
