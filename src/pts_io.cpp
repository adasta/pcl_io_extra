/*
 * pts_io.cpp
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */

#include <pcl/io/pts_io.h>
#include <istream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

pcl::PTSReader::PTSReader()
{
}



pcl::PTSReader::~PTSReader()
{
}



int pcl::PTSReader::readHeader(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
                               Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                               int & file_version, int & data_type, unsigned int & data_idx,
                               const int offset)
{

  boost::filesystem::path fpath  = file_name;

  if ( !boost::filesystem::exists(fpath) ){
    pcl::console::print_error("[PTSReader] File %s does not exist.\n", file_name.c_str());
    return -1;
  }
  if ( fpath.extension().string() != ".pts"){
    pcl::console::print_error("[PTSReader] File does not have pts extension. \n");
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
  f.name="rgb";
  f.offset =12;
  cloud.fields.push_back(f);
  }
  cloud.point_step = 16;


  std::fstream ifile(file_name.c_str(), std::fstream::in);
  std::string line;
  int total=0;
  int cdown =0;
  while(std::getline(ifile,line)){
    cdown--;
    if (cdown <=0) {
      std::vector<std::string> tokens;
      boost::algorithm::split(tokens, line,boost::algorithm::is_any_of(" \n\r\t"), boost::algorithm::token_compress_on);
      if (tokens.size() <3){
        int count=0;
        if (sscanf(tokens[0].c_str(), "%d", &count) )total+= count;
        cdown = count;
      }
    }
    }
  cloud.height=1;
  cloud.width = total;
  ifile.close();

  return cloud.height * cloud.width;
}


int pcl::PTSReader::read(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
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
      std::vector<std::string> tokens;
      boost::algorithm::split(tokens, line,boost::algorithm::is_any_of(" \n\r\t"), boost::algorithm::token_compress_on);
      if (tokens.size() <6 ){
        continue;
      }
        std::stringstream ss(line);
       float x=0,y=0,z=0;
       int r=0, g=0, b=0;
       float pid;
       ss >> x ;
       ss >> y ;
       ss >> z ;
       ss >> pid;
       ss >> r ;
       ss >> g ;
       ss >>b;
       if ((x==0) && (y==0) && (z==0)){
         x=y=z = std::numeric_limits<float>::quiet_NaN();
       }
       float *data = ( (float *) ( cloud.data.data() + ptoffset) )  ;
       *( ( data    ) )  = x;
       *( ( data +1 ) )  =y;
       *( ( data +2 ) )  =z;
       uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)b<< 8 | (uint32_t)b);
       *( ( data+ 3) )  =  *reinterpret_cast<float*>(&rgb);
       ptoffset+= cloud.point_step;
    }
  return cloud.width*cloud.height;
}

