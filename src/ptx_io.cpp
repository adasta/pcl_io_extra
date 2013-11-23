/*
 * ptx_io.cpp
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */


#include <pcl/io/ptx_io.h>
#include <istream>
#include <fstream>
#include <boost/filesystem.hpp>

/*
 * The first two lines contain the number of scan lines (n) and the number of points per
scanline (m), respectively.
The next 8 lines consist of a 4x3 and followed by a 4x4 matrix. They are generally
unused but can be used to store a transformation. Ignore these lines.

Then come the points one per line.
The points are arranged in a grid: Every m points comprise a single scan line
and there are n*m points in total.

If the scan has no colors attached to the points then each line is of the form:

x y z i

where (x,y,z) is the points 3D coordinates expressed in meters and i is the intensity
of the returned laser beam (0 < i < 1).

If the scan has colors attached to the points then each point is of the form:

x y z i r g b

x y z and i are as before and r g b are the red, green, and blue color components (0 <= r,g,b <= 255).
The r,g,b values have been acquired by color camera attached on the scanner.

Some of the points have their (x,y,z) coordinates  equal to (0,0,0). These are points for which the scanner was not able to make any measurements
(for instance highly reflective surfaces, sky, etc.).

ptx description taken from 3D Urban Challenge (Ioannis Stamos of Hunter College of the City University of New York
  dataset README.

 */


pcl::PTXReader::PTXReader()
{
}



pcl::PTXReader::~PTXReader()
{
}



int pcl::PTXReader::readHeader(const std::string & file_name, pcl::PCLPointCloud2 & cloud,
                               Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                               int & file_version, int & data_type, unsigned int & data_idx,
                               const int offset)
{

  boost::filesystem::path fpath  = file_name;

  if ( !boost::filesystem::exists(fpath) ){
    pcl::console::print_error("[PTXReader] File %s does not exist.\n", file_name.c_str());
    return -1;
  }
  if ( fpath.extension().string() != ".ptx"){
    pcl::console::print_error("[PTXReader] File does not have ptx extension. \n");
    return -1;
  }

  std::fstream ifile(file_name.c_str(), std::fstream::in);
 int rows, cols;
 char line[100];
 ifile.getline(line,100);

 {std::stringstream ss(line); ss >> cols;}
 ifile.getline(line,100);
 {std::stringstream ss(line); ss >> rows;}

 cloud.width = cols;
 cloud.height= rows;
 cloud.is_dense =true;
 for(int i=0; i<4; i++) ifile.getline(line,100);

 Eigen::Matrix4f tf;
 for(int i=0; i<4; i++){
   ifile.getline(line,100);
   std::stringstream ss(line);
   for(int j=0; j< 4;j++) ss>> tf(i,j);
 }
 tf.transposeInPlace();
 orientation = tf.block<3,3>(0,0);
 origin = tf.block<4,1>(0,3);


 {
    pcl::PCLPointField f;
  f.datatype = pcl::PCLPointField::FLOAT32;
  f.count= 1;
  f.name="x";
  cloud.fields.push_back(f);
  }

  {
  pcl::PCLPointField f;
  f.datatype = pcl::PCLPointField::FLOAT32;
  f.count= 1;
  f.name="y";
  f.offset =4;
  cloud.fields.push_back(f);
  }
  {
  pcl::PCLPointField f;
  f.datatype = pcl::PCLPointField::FLOAT32;
  f.count= 1;
  f.name="z";
  f.offset =8;
  cloud.fields.push_back(f);
  }

  {
  pcl::PCLPointField f;
  f.datatype = pcl::PCLPointField::FLOAT32;
  f.count= 1;
  f.name="intensity";
  f.offset =12;
  cloud.fields.push_back(f);
  }


  {
  pcl::PCLPointField f;
  f.datatype = pcl::PCLPointField::FLOAT32;
  f.count= 1;
  f.name="rgb";
  f.offset =16;
  cloud.fields.push_back(f);
  }
  cloud.point_step = 20;
  ifile.close();
}


int pcl::PTXReader::read(const std::string & file_name, pcl::PCLPointCloud2 & cloud,
                         Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                         int & file_version, const int offset)
{

  int  data_type;
  unsigned int data_idx;
  if ( this->readHeader(file_name,cloud,origin,orientation,file_version, data_type, data_idx, offset) <0 ) return -1;
  cloud.data.resize(cloud.height*cloud.width*cloud.point_step);

  char line[100];
  std::fstream ifile(file_name.c_str(), std::fstream::in);

  for(int i=0; i<8; i++) ifile.getline(line,100);

  for(int c=0, i=0; c<cloud.width; c++){
    for(int ri=cloud.height-1; ri>=0; ri--, i++){
      ifile.getline(line,100);
       std::stringstream ss(line);
       float x=0,y=0,z=0, intensity=0;
       int r=0, g=0, b=0;
       ss >> x >> y >> z >> intensity >>  r >> g >>b;
       if ((x==0) && (y==0) && (z==0)){
         x=y=z = std::numeric_limits<float>::quiet_NaN();
       }
       uint32_t offset = (cloud.width*ri+c)*cloud.point_step;

       float *data = ( (float *) ( cloud.data.data() + offset) )  ;
       *( ( data    ) )  = x;
       *( ( data +1 ) )  =y;
       *( ( data +2 ) )  =z;
       *( ( data +3) )  =intensity;
       uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g<< 8 | (uint32_t)b);
       *( ( data+ 4) )  =  *reinterpret_cast<float*>(&rgb);
    }
  }
  return cloud.width*cloud.height;
}



pcl::PTXWriter::PTXWriter()
{
  //TODO implement ptx writer
}



pcl::PTXWriter::~PTXWriter()
{
}

int pcl::PTXWriter::write(const std::string& file_name,
		const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin,
		const Eigen::Quaternionf& orientation, const bool binary) {

std::ofstream ofile( file_name.c_str());
ofile << cloud.width << "\n"<< cloud.height << "\n";

Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
tf.block<3,3>(0,0) = orientation.toRotationMatrix() ;
tf.block<4,1>(0,3) = origin;

for( int c=0 ; c<4; c++){
	for(int r=0; r<2; r++){
		ofile << tf(r,c) << " ";
	}
	ofile << tf(2,c) << "\n";
}

tf = Eigen::Matrix4f::Identity();
ofile << tf << "\n";

pcl::PointCloud<pcl::PointXYZI> tmp;
pcl::fromPCLPointCloud2(cloud, tmp);
for(int i=0; i< tmp.size(); i++){
	ofile << tmp[i].x << " " << tmp[i].y << " " << tmp[i].z << " " << tmp[i].intensity << "\n";
}
/*
if (pcl::getFieldIndex(cloud,"rgba")){
	return -1;
	assert(false && "not implemented");
}
else if( pcl::getFieldIndex(cloud,"intensity")>=0 ){
}
else{
	assert(false && "not implemented");
	return -1;
}
*/

return cloud.width*cloud.height;
}
