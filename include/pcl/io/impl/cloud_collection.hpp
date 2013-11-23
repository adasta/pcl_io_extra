/*
 * cloud_collection.hpp
 *
 *  Created on: Feb 21, 2012
 *      Author: asher
 */

#ifndef CLOUD_COLLECTION_HPP_
#define CLOUD_COLLECTION_HPP_

#include "../cloud_collection.h"
#include <pcl/common/io.h>

#include <iostream>

namespace pcl{
  namespace io {
  template<typename PointT> typename pcl::PointCloud<PointT>::Ptr
  CloudCollection::getCloud(){

    typename pcl::PointCloud<PointT>::Ptr cloud;

    std::vector<pcl::PCLPointField> fields;
    std::string key ="";
    pcl::getFields<PointT>(fields);

    for(int i=0; i< fields.size(); i++){
      char tmp[30];
      sprintf(tmp, "%s-%d-%d_", fields[i].name.c_str(),fields[i].datatype, fields[i].count);
      key = key+tmp;
    }

    try
    {
      return boost::any_cast<typename pcl::PointCloud<PointT>::Ptr >(point_map_[key]);
    }
    catch(std::exception & e)
    {
     return cloud;
    }

  }

  template<typename PointT>
  void CloudCollection::getCloud(typename pcl::PointCloud<PointT>::Ptr& cloud){
  	    std::vector<pcl::PCLPointField> fields;
  	    std::string key ="";
  	    pcl::getFields<PointT>(fields);
  	    cloud.reset();
  	    for(int i=0; i< fields.size(); i++){
  	      char tmp[30];
  	      sprintf(tmp, "%s-%d-%d_", fields[i].name.c_str(),fields[i].datatype, fields[i].count);
  	      key = key+tmp;
  	    }

  	    try
  	    {
  	      cloud = boost::any_cast<typename pcl::PointCloud<PointT>::Ptr >(point_map_[key]);
  	    }
  	    catch(std::exception & e)
  	    {
  	    }
  }

  template<typename PointT> void
  CloudCollection::setCloud(typename pcl::PointCloud<PointT>::Ptr cloud){
        std::vector<pcl::PCLPointField> fields;
        std::string key ="";
        pcl::getFields<PointT>(fields);

        for(int i=0; i< fields.size(); i++){
          char tmp[30];
          sprintf(tmp, "%s-%d-%d_", fields[i].name.c_str(),fields[i].datatype, fields[i].count);
          key = key+tmp;
        }
        point_map_[key] = cloud;
  }
}
}
#endif /* CLOUD_COLLECTION_HPP_ */
