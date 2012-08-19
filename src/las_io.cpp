

#include <pcl/io/las_io.h>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include <fstream>  // std::ifstream
#include <iostream> // std::cout

pcl::LASReader::LASReader()
{

}

pcl::LASReader::~LASReader()
{
}

int pcl::LASReader::readHeader(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
                               Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                               int & file_version, int & data_type, unsigned int & data_idx, const int offset)
{
  std::ifstream ifs;
  ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);

  liblas::Reader reader(ifs);

  liblas::Header const& header = reader.GetHeader();

  //todo
}



/*
 *
Base properties of all points regardless of dataformat_id Name
x
y
z
intensity
return_number
number_of_returns
scan_direction
flightline_edge
classification
scan_angle
user_data

For PCL, I am just going to use x,y,z, itensity and colro
 */

int pcl::LASReader::read(const std::string & file_name,
                         sensor_msgs::PointCloud2 & cloud,
                         Eigen::Vector4f & origin,
                         Eigen::Quaternionf & orientation,
                         int & file_version, const int offset)
{
  std::ifstream ifs;
  ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);

  liblas::Reader reader(ifs);


  unsigned int idx = 0;

  unsigned int nr_points;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  {
    sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT64;
  f.count= 1;
  f.name="x";
  cloud.fields.push_back(f);
  }

  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT64;
  f.count= 1;
  f.name="y";
  f.offset =8;
  cloud.fields.push_back(f);
  }
  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT64;
  f.count= 1;
  f.name="z";
  f.offset =16;
  cloud.fields.push_back(f);
  }

  {
  sensor_msgs::PointField f;
  f.datatype = sensor_msgs::PointField::FLOAT32;
  f.count= 1;
  f.name="intensity";
  f.offset =24;
  cloud.fields.push_back(f);
  }

  const int point_size = 28;
  cloud.data.resize( reader.GetHeader().GetPointRecordsCount() * point_size);
  cloud.height =1;
  cloud.width = reader.GetHeader().GetPointRecordsCount();
  cloud.point_step =point_size;

  for(uint64_t i=0; reader.ReadNextPoint(); i++)
  {
      liblas::Point const& p = reader.GetPoint();
       *( (double *) ( cloud.data.data() +point_size*i     ) )=  p.GetX();
       *( (double *) ( cloud.data.data() + point_size*i +8 ) )=  p.GetY();
       *( (double *) ( cloud.data.data() + point_size*i +16 ) )=  p.GetZ();
       *( (float *) ( cloud.data.data() + point_size*i +24) ) = p.GetIntensity();
  }
  return cloud.width*cloud.height;
}

pcl::LASWriter::LASWriter()
{
}


pcl::LASWriter::~LASWriter()
{
}


int pcl::LASWriter::write(const std::string & file_name, const sensor_msgs::PointCloud2 & cloud, const Eigen::Vector4f & origin, const Eigen::Quaternionf & orientation, const bool binary)
{
}

