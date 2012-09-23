/*
 * intensity_point_type.h
 *
 *  Created on: Jul 20, 2012
 *      Author: Adam Stambler
 */

#ifndef EXTRA_POINT_TYPE_H_
#define EXTRA_POINT_TYPE_H_

#include <pcl/point_types.h>


namespace pcl{
  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
    * \ingroup common
    */
 // struct  Intensity
 // {
 //       float intensity;
  // };

  struct PointXYZD{
      union {
         double data[3]; \
         struct {
           double x;
           double y;
           double z;
         };
       };
       inline Eigen::Map<Eigen::Vector3d> getVector3dMap () { return (Eigen::Vector3d::Map (data)); }
       inline const Eigen::Map<const Eigen::Vector3d> getVector3dMap () const { return (Eigen::Vector3d::Map (data)); }
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT ( pcl::PointXYZD,
                                    (double, x, x)
                                    (double, y, y)
                                    (double, z, z)
                                    )


//POINT_CLOUD_REGISTER_POINT_STRUCT ( pcl::Intensity,
   //                               (float, intensity, intensity)
     //                             )



#endif /* INTENSITY_POINT_TYPE_H_ */
