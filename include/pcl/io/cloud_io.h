/*
 * CloudReader.h
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */

#ifndef CLOUDREADER_H_
#define CLOUDREADER_H_

#include <pcl/io/file_io.h>

#include <map>
#include <string>

namespace pcl
{
  namespace io
  {

    class CloudReader : public FileReader
    {
    public:
      CloudReader ();
      virtual
      ~CloudReader ();

      /*
       * Register a file extension for the reader.  This is meant to be a wrapper around
       * a bunch of supported pont cloud types.
       * only pcd is default
       * CloudReader handles deletion of file reader pointers
       */
      bool registerExtension(std::string ext, FileReader* reader);

      void getSupportedExtensions(std::vector<std::string> extensions);

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
           readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                       Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                       int &file_version, int &data_type, unsigned int &data_idx, const int offset = 0);

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
           read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                 Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &file_version,
                 const int offset = 0);


      /** \brief Read a point cloud data from any FILE file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */

 template<typename PointT> inline int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud, const int offset  =0){
   FileReader::read(file_name, cloud, offset);
 }
      
      
    private:
      /*
       * reader_map_ is a map of point cloud file extensions to its associated
       * file reader implementation pointer.
       */
      std::map<std::string, FileReader*> reader_map_;

      //Analyzes file name extension to determine correct file reader;
      FileReader* chooseReader(std::string file_name);
    };

  } /* namespace io */
} /* namespace pcl */
#endif /* CLOUDREADER_H_ */
