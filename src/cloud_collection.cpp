/*
 * CloudCollection.cpp
 *
 *  Created on: Feb 21, 2012
 *      Author: asher
 */

#include <pcl/io/cloud_collection.h>
#include <pcl/io/points_to_rosmsg.h>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <pcl/extra_point_type.h>


namespace pcl
{
  namespace io{
  CloudCollection::CloudCollection ()
  {
    // TODO Auto-generated constructor stub

  }

  CloudCollection::~CloudCollection ()
  {
    // TODO Auto-generated destructor stub
  }

  void
  CloudCollection::getCloudList(std::vector<std::string>& clouds){
    clouds.clear();
    for(std::map<std::string, boost::any>::iterator iter = point_map_.begin();
        iter!= point_map_.end(); iter++){
      clouds.push_back(iter->first);
    }
  }

  bool
  CloudCollection::load(const std::string& name){

    pcl::PCLPointCloud2::Ptr cloud;
    cloud.reset(new pcl::PCLPointCloud2);
    Eigen::Vector4f origin;
    Eigen::Quaternionf rot;
    if ( pcl::io::loadPCDFile (name, *cloud, origin, rot) <0) return false;

    for (int i=0; i<cloud->fields.size(); i++ ){
          if ( strcmp(cloud->fields[i].name.c_str(), "x") ==0 ){
               pcl::PointCloud<pcl::PointXYZ>::Ptr xyz( new pcl::PointCloud<pcl::PointXYZ>);
               xyz->sensor_origin_ = origin;
			   xyz->sensor_orientation_ = rot;
               setCloud<pcl::PointXYZ>(xyz);
               pcl::fromPCLPointCloud2(*cloud, *xyz);
           }
            if ( strcmp(cloud->fields[i].name.c_str(), "rgb") ==0  || strcmp(cloud->fields[i].name.c_str(), "rgba") ==0 ){
              pcl::PointCloud<pcl::RGB>::Ptr rgb( new pcl::PointCloud<pcl::RGB>);
              setCloud<pcl::RGB>(rgb);
              pcl::fromPCLPointCloud2(*cloud, *rgb);
            }
            if ( strcmp(cloud->fields[i].name.c_str(), "normal_x") ==0 ){
              pcl::PointCloud<pcl::Normal>::Ptr normal( new pcl::PointCloud<pcl::Normal>);
                setCloud<pcl::Normal>(normal);
               pcl::fromPCLPointCloud2(*cloud, *normal);
            }
            if ( strcmp(cloud->fields[i].name.c_str(), "label") ==0 ){
               pcl::PointCloud<pcl::Label>::Ptr label( new pcl::PointCloud<pcl::Label>);
               setCloud<pcl::Label>(label);
               pcl::fromPCLPointCloud2(*cloud, *label);
            }
            if ( strcmp(cloud->fields[i].name.c_str(), "intensity") ==0 ){
               pcl::PointCloud<pcl::Intensity>::Ptr label( new pcl::PointCloud<pcl::Intensity>);
               setCloud<pcl::Intensity>(label);
               pcl::fromPCLPointCloud2(*cloud, *label);
            }

    }
    return true;
  }


  void CloudCollection::save(const std::string& name){
  pcl::PCLPointCloud2 cloud_blob;
  getPointCloud2(cloud_blob);

  pcl::PCDWriter writer;
  if (cloud_blob.width) writer.writeBinaryCompressed(name, cloud_blob,
     this->getCloud<pcl::PointXYZ>()->sensor_origin_,
     this->getCloud<pcl::PointXYZ>()->sensor_orientation_);
}

  void  CloudCollection::getPointCloud2(pcl::PCLPointCloud2 & cloud_blob)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz = getCloud<pcl::PointXYZ>();
    pcl::toPCLPointCloud2(*xyz, cloud_blob);

    pcl::PointCloud<pcl::Normal>::Ptr n = this->getCloud<pcl::Normal>();
    pcl::PointCloud<pcl::RGB>::Ptr r = this->getCloud<pcl::RGB>();
    pcl::PointCloud<pcl::Label>::Ptr l = this->getCloud<pcl::Label>();
    pcl::PointCloud<pcl::Intensity>::Ptr ic = this->getCloud<pcl::Intensity>();

    if ( n != NULL){
      pcl::PCLPointCloud2 tmp, tmp2;
      tmp2 = cloud_blob;
      pcl::toPCLPointCloud2(*n, tmp);
      pcl::concatenateFields(tmp, tmp2, cloud_blob);
    }
    if ( r != NULL){
        pcl::PCLPointCloud2 tmp, tmp2;
        tmp2 = cloud_blob;
        pcl::toPCLPointCloud2(*r, tmp);
        pcl::concatenateFields(tmp, tmp2, cloud_blob);
      }
    if ( ic != NULL){
      pcl::PCLPointCloud2 tmp, tmp2;
      tmp2 = cloud_blob;
      pcl::toPCLPointCloud2(*ic, tmp);
      pcl::concatenateFields(tmp, tmp2, cloud_blob);
    }
    if ( l != NULL){
      pcl::PCLPointCloud2 tmp, tmp2;
      tmp2 = cloud_blob;
      pcl::toPCLPointCloud2(*l, tmp);
      pcl::concatenateFields(tmp, tmp2, cloud_blob);
    }
  }

  }
}
 /* namespace pcl */
