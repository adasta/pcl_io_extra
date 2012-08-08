/*
 * CloudReader.cpp
 *
 *  Created on: May 21, 2012
 *      Author: asher
 */

#include <pcl/io/cloud_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ptx_io.h>
#include <pcl/io/pts_io.h>
#include <algorithm>



namespace pcl
{
    CloudReader::CloudReader ()
    {
      // TODO Auto-generated constructor stub

      reader_map_["pcd"] = new pcl::PCDReader;
      reader_map_["ptx"] = new pcl::PTXReader;
      reader_map_["pts"] = new pcl::PTSReader;

    }

    CloudReader::~CloudReader ()
    {
      // TODO Auto-generated destructor stub
      for(std::map<std::string, FileReader*>::iterator iter= reader_map_.begin();
          iter!= reader_map_.end(); iter++){
        delete iter->second;
      }
    }

    bool CloudReader::registerExtension(std::string ext, FileReader *reader)
    {
      //TODO check for . and remove it

      reader_map_[ext] = reader;
      return true;
    }

      void CloudReader::getSupportedExtensions(std::vector<std::string> extensions)
      {
        for(std::map<std::string, FileReader*>::iterator iter= reader_map_.begin();
          iter!= reader_map_.end(); iter++){
          extensions.push_back(iter->first);
        }
      }

      int CloudReader::readHeader(const std::string & file_name, sensor_msgs::PointCloud2 & cloud, Eigen::Vector4f & origin, Eigen::Quaternionf & orientation, int & file_version, int & data_type, unsigned int & data_idx, const int offset)
      {
        FileReader* reader = chooseReader(file_name);
        if (reader == NULL) return -1;
        return reader->readHeader(file_name, cloud,origin,orientation,file_version, data_type, data_idx, offset);
      }

      int CloudReader::read(const std::string & file_name, sensor_msgs::PointCloud2 & cloud, Eigen::Vector4f & origin, Eigen::Quaternionf & orientation, int & file_version, const int offset)
      {
        FileReader* reader = chooseReader(file_name);
        if (reader == NULL) return -1;
        return reader->read(file_name, cloud, origin, orientation, file_version, offset);
      }

      FileReader *CloudReader::chooseReader(std::string file_name)
      {
        //figure out the extension
         size_t pos = file_name.rfind('.');
         if (pos == std::string::npos) {
           pcl::console::print_error("[CloudReader] Input file %s does not have extension.\n", file_name.c_str());
           return NULL;
         }
         std::string ext = file_name.substr(pos+1,file_name.size()-pos);
         size_t p2 = ext.find_last_not_of( " \t\n\r" );
         if( std::string::npos != p2 )
           ext.erase( p2+1 );
         else
           ext.clear();

         if (reader_map_.count(ext) ==0 ) {
           pcl::console::print_error("[CloudReader] %s extension not supported.\n", ext.c_str());
           return NULL;
         }
         return reader_map_[ext];
      }
} /* namespace pcl */
