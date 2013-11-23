/*
 * io_util.h
 *
 *  Created on: Jan 11, 2012
 *      Author: asher
 */

#ifndef IO_UTIL_H_
#define IO_UTIL_H_

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <cstddef>

#include <pcl/pcl_macros.h>
#include <pcl/exceptions.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <boost/mpl/size.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#include <boost/foreach.hpp>


#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

namespace pcl {
  /** \brief Convert two pcl::PointCloud<T> objects to a PointCloud2 binary data blob.
    * \param[in] cloud the input pcl::PointCloud<T>
    * \param[out] msg the resultant PointCloud2 binary blob
    */

  template<typename PointA, typename PointB> void
  pointsToROSMsg (const pcl::PointCloud<PointA>& cloudA, const pcl::PointCloud<PointB>& cloudB,
                  pcl::PCLPointCloud2& msg)
  {
    assert(cloudA.points.size() == cloudB.points.size());

    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloudA.width == 0 && cloudA.height == 0)
    {
      msg.width  = (uint32_t) cloudA.points.size ();
      msg.height = 1;
    }
    else
    {
      assert (cloudA.points.size () == cloudA.width * cloudA.height);
      msg.height = cloudA.height;
      msg.width  = cloudA.width;
    }

    // Fill point cloud binary data (padding and all)
    size_t point_size =sizeof (PointA) + sizeof (PointB);
    size_t data_size = point_size * cloudA.points.size ();
    msg.data.resize (data_size);
    for(int i=0; i< cloudA.points.size(); i++){
      memcpy (&msg.data[i*point_size], &cloudA.points[i], sizeof (PointA));
      memcpy (&msg.data[i*point_size+sizeof(PointA)], &cloudB.points[i], sizeof (PointB));
    }

    // Fill fields metadata
    msg.fields.clear ();
    for_each_type< typename traits::fieldList<PointA>::type > (detail::FieldAdder<PointA>(msg.fields));
    int aF = msg.fields.size();
    for_each_type< typename traits::fieldList<PointB>::type > (detail::FieldAdder<PointB>(msg.fields));
    for(int i=aF; i<msg.fields.size(); i++) msg.fields[i].offset += sizeof(PointA);

    msg.header     = cloudA.header;
    msg.point_step = point_size;
    msg.row_step   = point_size * msg.width;
    msg.is_dense   = cloudA.is_dense;
    /// @todo msg.is_bigendian = ?;
  }


  template<typename PointA, typename PointB, typename PointC> void
  pointsToROSMsg (const pcl::PointCloud<PointA>& cloudA, const pcl::PointCloud<PointB>& cloudB,
                  const pcl::PointCloud<PointC>& cloudC,
                  pcl::PCLPointCloud2& msg)
  {
    assert((cloudA.points.size() == cloudB.points.size())
           && (cloudA.points.size()== cloudC.points.size() ));

    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloudA.width == 0 && cloudA.height == 0)
    {
      msg.width  = (uint32_t) cloudA.points.size ();
      msg.height = 1;
    }
    else
    {
      assert (cloudA.points.size () == cloudA.width * cloudA.height);
      msg.height = cloudA.height;
      msg.width  = cloudA.width;
    }

    // Fill point cloud binary data (padding and all)
    size_t point_size =sizeof (PointA) + sizeof (PointB)+sizeof(PointC);
    size_t data_size = point_size * cloudA.points.size ();
    msg.data.resize (data_size);
    for(int i=0; i< cloudA.points.size(); i++){
      memcpy (&msg.data[i*point_size], &cloudA.points[i], sizeof (PointA));
      memcpy (&msg.data[i*point_size+sizeof(PointA)], &cloudB.points[i], sizeof (PointB));
      memcpy (&msg.data[i*point_size+sizeof(PointA)+sizeof(PointB)], &cloudC.points[i], sizeof (PointC));

    }

    // Fill fields metadata
    msg.fields.clear ();
    for_each_type< typename traits::fieldList<PointA>::type > (detail::FieldAdder<PointA>(msg.fields));
    int Fs = msg.fields.size();
    for_each_type< typename traits::fieldList<PointB>::type > (detail::FieldAdder<PointB>(msg.fields));
    for(int i=Fs; i<msg.fields.size(); i++) msg.fields[i].offset += sizeof(PointA);
    Fs = msg.fields.size();
    for_each_type< typename traits::fieldList<PointC>::type > (detail::FieldAdder<PointC>(msg.fields));
    for(int i=Fs; i<msg.fields.size(); i++) msg.fields[i].offset += sizeof(PointA) + sizeof(PointB);

    msg.header     = cloudA.header;
    msg.point_step = point_size;
    msg.row_step   = point_size * msg.width;
    msg.is_dense   = cloudA.is_dense;
    /// @todo msg.is_bigendian = ?;
  }


  template<typename PointA, typename PointB, typename PointC, typename PointD> void
  pointsToROSMsg (const pcl::PointCloud<PointA>& cloudA, const pcl::PointCloud<PointB>& cloudB,
                  const pcl::PointCloud<PointC>& cloudC, const pcl::PointCloud<PointD>& cloudD,
                  pcl::PCLPointCloud2& msg)
  {
    assert( (cloudA.points.size() == cloudB.points.size()) &&
            (cloudC.points.size() == cloudD.points.size()) &&
            (cloudB.points.size() == cloudC.points.size()));

    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloudA.width == 0 && cloudA.height == 0)
    {
      msg.width  = (uint32_t) cloudA.points.size ();
      msg.height = 1;
    }
    else
    {
      assert (cloudA.points.size () == cloudA.width * cloudA.height);
      msg.height = cloudA.height;
      msg.width  = cloudA.width;
    }

    // Fill point cloud binary data (padding and all)
    size_t point_size =sizeof (PointA) + sizeof (PointB)+sizeof(PointC)+sizeof(PointD);
    size_t data_size = point_size * cloudA.points.size ();
    msg.data.resize (data_size);
    int Coffset = sizeof(PointA)+sizeof(PointB);
    int Doffset = sizeof(PointA)+sizeof(PointB)+sizeof(PointC);
    for(int i=0; i< cloudA.points.size(); i++){
      memcpy (&msg.data[i*point_size], &cloudA.points[i], sizeof (PointA));
      memcpy (&msg.data[i*point_size+sizeof(PointA)], &cloudB.points[i], sizeof (PointB));
      memcpy (&msg.data[i*point_size+Coffset], &cloudC.points[i], sizeof (PointC));
      memcpy (&msg.data[i*point_size+Doffset], &cloudD.points[i], sizeof (PointD));
    }

    // Fill fields metadata
    msg.fields.clear ();
    for_each_type< typename traits::fieldList<PointA>::type > (detail::FieldAdder<PointA>(msg.fields));
    int Fs = msg.fields.size();
    for_each_type< typename traits::fieldList<PointB>::type > (detail::FieldAdder<PointB>(msg.fields));
    for(int i=Fs; i<msg.fields.size(); i++) msg.fields[i].offset += sizeof(PointA);
    Fs = msg.fields.size();
    for_each_type< typename traits::fieldList<PointC>::type > (detail::FieldAdder<PointC>(msg.fields));
    for(int i=Fs; i<msg.fields.size(); i++) msg.fields[i].offset += Coffset;
    Fs = msg.fields.size();
    for_each_type< typename traits::fieldList<PointD>::type > (detail::FieldAdder<PointD>(msg.fields));
    for(int i=Fs; i<msg.fields.size(); i++) msg.fields[i].offset += Doffset;

    msg.header     = cloudA.header;
    msg.point_step = point_size;
    msg.row_step   = point_size * msg.width;
    msg.is_dense   = cloudA.is_dense;
    /// @todo msg.is_bigendian = ?;
  }


}
#endif /* IO_UTIL_H_ */
