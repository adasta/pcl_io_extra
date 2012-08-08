/*
 * intensity_point_type.h
 *
 *  Created on: Jul 20, 2012
 *      Author: Adam Stambler
 */

#ifndef INTENSITY_POINT_TYPE_H_
#define INTENSITY_POINT_TYPE_H_

#include <pcl/point_types.h>

namespace pcl{
  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
    * \ingroup common
    */
  struct  Intensity
  {
        float intensity;
   };
}

POINT_CLOUD_REGISTER_POINT_STRUCT ( pcl::Intensity,
                                  (float, intensity, intensity)
                                  )



#endif /* INTENSITY_POINT_TYPE_H_ */
