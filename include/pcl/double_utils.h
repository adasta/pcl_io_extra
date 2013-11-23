/*
 * double.h
 *
 *  Created on: Aug 8, 2012
 *      Author: Adam Stambler
 */

#ifndef DOUBLE_UTILS_H_
#define DOUBLE_UTILS_H_

#include <Eigen/Core>
#include <pcl/PCLPointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl{


bool hasDoublePointXYZ(const pcl::PCLPointCloud2& cloud);


template<typename PointTA, typename PointTB>
void cvtAndOffset( const pcl::PointCloud<PointTA>& cloudA, const pcl::PointCloud<PointTB>& cloudB, Eigen::Vector3d& offset){
    if ( cloudB.points.size() != cloudA.points.size()){
      cloudB.points.resize(cloudA.size());
    }
    for(int i=0; i<cloudA.points.size(); i++){
      cloudB.points[i].x = cloudA.points[i].x+offset[0];
      cloudB.points[i].y = cloudA.points[i].y+offset[1];
      cloudB.points[i].z = cloudA.points[i].z+offset[2];
    }
    cloudB.header = cloudA.header;
    cloudB.width = cloudA.width;
    cloudB.height=cloudA.height;
  }


bool cvtToDoubleAndOffset( const pcl::PCLPointCloud2& cloudA, pcl::PCLPointCloud2& cloudB,
    Eigen::Vector3d& offset);

}

#endif /* DOUBLE_H_ */
