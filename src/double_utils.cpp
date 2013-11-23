/*
 * double_utils.cpp
 *
 *  Created on: Aug 8, 2012
 *      Author: Adam Stambler
 */


#include <pcl/double_utils.h>

bool pcl::hasDoublePointXYZ (const pcl::PCLPointCloud2& cloud)
{
  for(int i=0; i<cloud.fields.size(); i++){
    bool is_coord=false;
    if ((cloud.fields[i].name== "x") && (cloud.fields[i].datatype == pcl::PCLPointField::FLOAT64) ){
      return true;
    }
    if ((cloud.fields[i].name== "y") && (cloud.fields[i].datatype == pcl::PCLPointField::FLOAT64) ){
      return true;
    }
    if ((cloud.fields[i].name== "z") && (cloud.fields[i].datatype == pcl::PCLPointField::FLOAT64) ){
      return true;
    }
  }
}

bool pcl::cvtToDoubleAndOffset( const pcl::PCLPointCloud2& cloudA, pcl::PCLPointCloud2& cloudB, Eigen::Vector3d& offset){

  cloudB.fields.clear();

  int xidx = -1,yidx=-1, zidx =-1;
  int byte_offset=0;
  std::vector<int> copy_idx;
  std::vector<int> length;
  int point_size=0;
  for(int i=0; i<cloudA.fields.size(); i++){
    bool is_coord=false;
    if (cloudA.fields[i].name== "x"){
      xidx=i;
      is_coord=true;
    }
    else if (cloudA.fields[i].name== "y"){
      yidx=i;
      is_coord=true;
    }
    else if (cloudA.fields[i].name== "z"){
      zidx=i;
      is_coord=true;
    }
    if(is_coord){
      cloudB.fields.push_back(cloudA.fields[i]);
      cloudB.fields[i].datatype = pcl::PCLPointField::FLOAT32;
      cloudB.fields[i].count=1;
      cloudB.fields[i].offset = byte_offset;
      byte_offset+=4;
      point_size+=4;
    }
    else{
      cloudB.fields.push_back(cloudA.fields[i]);
      cloudB.fields[i].offset = byte_offset;
      copy_idx.push_back(i);
      switch (cloudB.fields.back().datatype){
        case pcl::PCLPointField::FLOAT64:
          byte_offset+=8;
          point_size+=8;
          length.push_back(8);
          break;
        case pcl::PCLPointField::FLOAT32:
        case pcl::PCLPointField::INT32:
        case pcl::PCLPointField::UINT32:
          byte_offset+=4;
          point_size+= 4;
          length.push_back(4);
          break;
        case pcl::PCLPointField::UINT16:
        case pcl::PCLPointField::INT16:
          byte_offset+=2;
          point_size+= 2;
          length.push_back(2);
          break;
        case pcl::PCLPointField::INT8:
        case pcl::PCLPointField::UINT8:
          byte_offset+=1;
          byte_offset+=1;
          length.push_back(1);
          break;
      }
    }
  }

  if ( (xidx<0) || (yidx <0) || (zidx<0) ) return false;

  cloudB.point_step = point_size;
  cloudB.width =cloudA.width;
  cloudB.height=cloudA.height;
  cloudB.header=cloudA.header;


  int npts = cloudB.width*cloudB.height;
  cloudB.data.resize(npts*point_size);

  uint8_t* dataB = cloudB.data.data();
  const uint8_t* dataA = cloudA.data.data();

  for(int i=0; i<npts; i++){
   *( (float*) (dataB +cloudB.fields[xidx].offset))= *((double *)(dataA+ cloudA.fields[xidx].offset)) + offset[0];
   *( (float*) (dataB +cloudB.fields[yidx].offset)) =*((double *)(dataA+ cloudA.fields[yidx].offset))  + offset[1];
   *( (float*) (dataB +cloudB.fields[zidx].offset)) =*((double *)(dataA+ cloudA.fields[zidx].offset))  + offset[2];

    for(int j=0; j<copy_idx.size(); j++){
      memcpy( dataB +cloudB.fields[copy_idx[j]].offset, dataA+ cloudA.fields[copy_idx[j]].offset, length[j]);
    }
    dataB+= point_size;
    dataA += cloudA.point_step;
  }
  return true;
  }
