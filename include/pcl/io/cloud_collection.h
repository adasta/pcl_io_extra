/*
 * CloudCollection.h
 *
 *  Created on: Feb 21, 2012
 *      Author: asher
 */

#ifndef CLOUDCOLLECTION_H_
#define CLOUDCOLLECTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/any.hpp>
#include <map>
#include <string>


namespace pcl {
  namespace traits{
  template<typename T> struct pointName {static const std::string name;};
  }
}

namespace pcl
{
  namespace io{
  class CloudCollection
  {
  public:
    CloudCollection ();
    virtual
    ~CloudCollection ();

    typedef boost::shared_ptr<CloudCollection> Ptr;

    template<typename PointT> typename pcl::PointCloud<PointT>::Ptr
    getCloud();

    template<typename PointT> void
    setCloud(typename pcl::PointCloud<PointT>::Ptr);

    void getPointCloud2(sensor_msgs::PointCloud2& cloud);

    void getCloudList(std::vector<std::string>& clouds);

    void load(const std::string& name);
    void save(const std::string& name);

  protected:
    std::map<std::string, boost::any> point_map_;

  };
  } // namespace io
} /* namespace pcl */

#include "impl/cloud_collection.hpp"
#endif /* CLOUDCOLLECTION_H_ */
