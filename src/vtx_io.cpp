/*
 * pts_io.cpp
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */

#include <pcl/io/vtx_io.h>
#include <istream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

pcl::VTXReader::VTXReader()
{
}



pcl::VTXReader::~VTXReader()
{
}



int pcl::VTXReader::readHeader(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
                               Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                               int & file_version, int & data_type, unsigned int & data_idx,
                               const int offset)
{

  boost::filesystem::path fpath  = file_name;

  if ( !boost::filesystem::exists(fpath) ){
    pcl::console::print_error("[VTXReader] File %s does not exist.\n", file_name.c_str());
    return -1;
  }
  if ( fpath.extension().string() != ".vtx"){
    pcl::console::print_error("[VTXReader] File does not have vtx extension. \n");
    return -1;
  }

  origin = Eigen::Vector4f::Zero();
  orientation = Eigen::Quaternionf();

 {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="x";
  cloud.fields.push_back(f);
  }

  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="y";
  f.offset =4;
  cloud.fields.push_back(f);
  }
  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="z";
  f.offset =8;
  cloud.fields.push_back(f);
  }

  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="normal_x";
  f.offset =12;
  cloud.fields.push_back(f);
  }

  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="normal_y";
  f.offset =16;
  cloud.fields.push_back(f);
  }

  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="normal_z";
  f.offset =20;
  cloud.fields.push_back(f);
  }


 cloud.point_step= 24;

  std::fstream ifile(file_name.c_str(), std::fstream::in);
  std::string line;
  int total=0;
  while(std::getline(ifile,line)){
    boost::algorithm::trim(line);
    if (line.find_first_not_of("#") !=0 ) continue;
    std::vector<std::string> tokens;
    boost::algorithm::split(tokens, line,boost::algorithm::is_any_of(", \n\r\t"), boost::algorithm::token_compress_on);
    int s = tokens.size();
    if (tokens.size() ==6)
      total++;
  }
  cloud.height=1;
  cloud.width = total;
  ifile.close();

  origin = Eigen::Vector4f();
  orientation = Eigen::Quaternionf();

  return cloud.height * cloud.width;
}


int pcl::VTXReader::read(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
                         Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                         int & file_version, const int offset)
{

  int  data_type;
  unsigned int data_idx;
  if ( this->readHeader(file_name,cloud,origin,orientation,file_version, data_type, data_idx, offset) <0 ) return -1;
  cloud.data.resize(cloud.height*cloud.width*cloud.point_step);

  std::string line;
  std::fstream ifile(file_name.c_str(), std::fstream::in);

    int total=0;

    uint32_t ptoffset=0;
    while(std::getline(ifile,line)){
      boost::algorithm::trim(line);
      if (line.find_first_not_of("#") !=0 ) continue;
         std::vector<std::string> tokens;
         boost::algorithm::split(tokens, line,boost::algorithm::is_any_of(", \n\r\t"), boost::algorithm::token_compress_on);
         if (tokens.size() !=6) continue;
         std::stringstream ss(line);
        float x=0,y=0,z=0;
        float nx=0, ny=0, nz=0;
        ss >> y ;
        ss >> z ;
        ss >> nx ;
        ss >> ny ;
        ss >> nz;
        if ((x==0) && (y==0) && (z==0)){
          x=y=z = std::numeric_limits<float>::quiet_NaN();
        }
        float *data = ( (float *) ( cloud.data.data() + ptoffset) )  ;
        *( ( data    ) )  = x;
        *( ( data +1 ) )  =y;
        *( ( data +2 ) )  =z;
        *( ( data  +3  ) )  = nx;
        *( ( data +4 ) )  =ny;
        *( ( data +5 ) )  =nz;

        ptoffset+= cloud.point_step;
    }
  return cloud.width*cloud.height;
}

