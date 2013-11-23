/*
 * FileReaderD.h
 *
 *  Created on: Jun 6, 2012
 *      Author: asher
 */

#ifndef FILEREADERD_H_
#define FILEREADERD_H_

#include <pcl/io/file_io.h>

/*
 * This class is a convience class that allows for easy manipulation of common point cloud formats
 * whose points may need more significant digits than a float number can handle.  This is common
 * for arial lidar or large scale terrestrial laser scans.  The hack is to allow reading of the
 * sensor origin to an eigen double.  Additionally, point offset can be set to reduce the significant
 * digits so they can be manipulated by floats.
 */


namespace pcl {

class FileReaderD  : public FileReader {
public:
	FileReaderD():_minoffset(false), _use_offset(false), _ox(0), _oy(0), _oz(0){}
	virtual ~FileReaderD(){}

	void setOffset(double ox, double oy, double oz){ _use_offset =true; _ox=ox; _oy=oy; _oz=oz;}
	void getOffset(double& x, double& y, double&z){x=_ox; y=_oy; z=_oz;}


	void setMinimumOffset(bool enable = true){_minoffset = enable;_use_offset = enable;};


	 /** \brief Read a point cloud data header from a FILE file and use double precision.
	        *
	        * Load only the meta information (number of points, their types, etc),
	        * and not the points themselves, from a given FILE file. Useful for fast
	        * evaluation of the underlying data structure.
	        *
	        * Returns:
	        *  * < 0 (-1) on error
	        *  * > 0 on success
	        * \param[in] file_name the name of the file to load
	        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
	        * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
	        * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
	        * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
	        * \param[out] data_type the type of data (binary data=1, ascii=0, etc)
	        * \param[out] data_idx the offset of cloud data within the file
	        * \param[in] offset the offset in the file where to expect the true header to begin.
	        * One usage example for setting the offset parameter is for reading
	        * data from a TAR "archive containing multiple files: TAR files always
	        * add a 512 byte header in front of the actual file, so set the offset
	        * to the next byte after the header (e.g., 513).
	        */
	      virtual int
	      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
	                  Eigen::Vector4d &origin, Eigen::Quaterniond &orientation,
	                  int &file_version, int &data_type, unsigned int &data_idx, const int offset = 0) = 0;


	 /** \brief Read a point cloud data header from a FILE file.
	        *
	        * Load only the meta information (number of points, their types, etc),
	        * and not the points themselves, from a given FILE file. Useful for fast
	        * evaluation of the underlying data structure.
	        *
	        * Returns:
	        *  * < 0 (-1) on error
	        *  * > 0 on success
	        * \param[in] file_name the name of the file to load
	        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
	        * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
	        * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
	        * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
	        * \param[out] data_type the type of data (binary data=1, ascii=0, etc)
	        * \param[out] data_idx the offset of cloud data within the file
	        * \param[in] offset the offset in the file where to expect the true header to begin.
	        * One usage example for setting the offset parameter is for reading
	        * data from a TAR "archive containing multiple files: TAR files always
	        * add a 512 byte header in front of the actual file, so set the offset
	        * to the next byte after the header (e.g., 513).
	        */
	      virtual int
	      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
	                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
	                  int &file_version, int &data_type, unsigned int &data_idx, const int offset = 0){
	    	  Eigen::Vector4d orig;
	    	  Eigen::Quaterniond orien;
	    	  int result = readHeader(file_name, cloud, orig, orien, file_version, data_type, data_idx, offset);
	    	  origin = orig.cast<float>();
	    	  orientation = orien.cast<float>();
	    	  return result;
	      }

	      /** \brief Read a point cloud data from a FILE file and store it into a sensor_msgs/PointCloud2.
	        * \param[in] file_name the name of the file containing the actual PointCloud data
	        * \param[out] cloud the resultant PointCloud message read from disk
	        * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
	        * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
	        * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
	        * \param[in] offset the offset in the file where to expect the true header to begin.
	        * One usage example for setting the offset parameter is for reading
	        * data from a TAR "archive containing multiple files: TAR files always
	        * add a 512 byte header in front of the actual file, so set the offset
	        * to the next byte after the header (e.g., 513).
	        */
	      virtual int
	      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
	            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &file_version,
	            const int offset = 0) = 0;

protected:
	//Offset all points by location by the minimum point
	bool _minoffset;
	//Use a point offset specified by o{x,y,z}
	bool _use_offset;

	//point offsets
	double _ox, _oy, _oz;
};

} /* namespace pcl */
#endif /* FILEREADERD_H_ */
