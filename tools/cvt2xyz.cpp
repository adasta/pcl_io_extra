

#include <iostream>
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ptx_io.h>

namespace po=boost::program_options;

int main(int argc, char** argv){
  po::options_description desc("./cvt2xyz input.pcd output.xyz\n converts PointXYZI clouds to ptx format\n");
  std::string infile;
  std::string ofile;

  desc.add_options()
    ("help", "produce help message")
    ("input,i",po::value<std::string>(&infile)->required(), "input point cloud ")
    ("output,o",po::value<std::string>(&ofile)->required(), "output xyz ")
          ;

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
 if (vm.count("help") ){
   std::cout << desc << std::endl;
   return 0;
 }

 pcl::PointCloud<pcl::PointXYZI> cloud;
 if(pcl::io::loadPCDFile(infile, cloud) <0 ){
	 std::cout << "Failed to read input \n";
	 return -1;
 }

std::ofstream osfile( ofile.c_str());

for(int i=0; i< cloud.size(); i++){
	osfile << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << "\n";
}

return 0;
}
