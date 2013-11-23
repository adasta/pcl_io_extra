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
#include <pcl/common/transforms.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pcl/double_utils.h>

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
          ("view_offset,v", po::value< std::string>(), "Offset view offset to translate and convert a float64 to float32 XYZ")
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
  std::cout << "Adding E57 Support : \n";
  reader.registerExtension("e57", new pcl::E57Reader( ));
#endif
#ifdef LAS
  reader.registerExtension("las", new pcl::LASReader );
#endif
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

  Eigen::Vector4f origin;
  Eigen::Quaternionf rot;

  int fv;
  std::clog << "Loading " <<  ifile << " \n";
  int c = reader.read(ifile,*cloud, origin, rot, fv);
  std::clog << "There were " << c << " points \n";

  if (pcl::hasDoublePointXYZ(*cloud) ){
    if (vm.count("view_offset")){
      Eigen::Vector3d voff;

      if ( 3 == sscanf(vm["view_offset"].as<std::string>().c_str(), "%lf,%lf,%lf", &voff[0], &voff[1], &voff[2]) ){
        std::cout << "Translating by " << voff.transpose() << " \n";
        pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
        pcl::cvtToDoubleAndOffset(*cloud,*cloud2,voff);
        cloud = cloud2;
      }
      else{
        std::cout << "Invalid view offset of : " << vm["view_offset"].as<std::string>() << " \n";
      }
    }
  }


  pcl::PCDWriter writer;
  writer.write(ofile, cloud,origin, rot, !vm.count("ascii"));
  std::cout << ofile << "\n";
}
