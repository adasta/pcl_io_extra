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
    pcl::PointCloud<PointT> tmp;
    std::string key =pcl::getFieldsList(tmp);
    try
    {
      return boost::any_cast<typename pcl::PointCloud<PointT>::Ptr >(point_map_[key]);
    }
    catch(boost::bad_any_cast & e)
    {
     return cloud;
    }

  }

  template<typename PointT> void
  CloudCollection::setCloud(typename pcl::PointCloud<PointT>::Ptr cloud){
    std::string key =pcl::getFieldsList(*cloud);
    point_map_[key] = cloud;
  }
}
}
#endif /* CLOUD_COLLECTION_HPP_ */
