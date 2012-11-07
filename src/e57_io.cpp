
#include <pcl/io/e57_io.h>
#include <e57/E57Foundation.h>
#include <boost/assign.hpp>


using namespace e57;
using namespace std;

/*
 * The FieldReader classes are just utility classes meant to match the
 * input buffer of the 57 reader to the point cloud binary data blob
 */
class FieldReaderBase{
  public:
  virtual ~FieldReaderBase(){};
  virtual void copy( int buffer_idx, int cloud_idx) =0;

  void setup( uint8_t* output, int point_size){
    odata_ = output;
    point_size_ = point_size;
  }
  protected:
    int point_size_;
    uint8_t* odata_;
};

template<class PCLT, class BUFFERT>
class FieldReader : public FieldReaderBase {
public:
  FieldReader(){
    offset =0;
  }
  FieldReader(const sensor_msgs::PointField& field, BUFFERT* buffer){
    field_ = field;
    buffer_ = buffer;
  }
 virtual  ~FieldReader(){}
  virtual void copy(int buffer_idx, int cloud_idx){
      *( (PCLT *) ( odata_ +point_size_*cloud_idx +field_.offset   ) )  =  buffer_[buffer_idx];
  }

  void setup( uint8_t* output, int point_size){
    odata_ = output;
    point_size_ = point_size;
  }

private:
  BUFFERT* buffer_;
  sensor_msgs::PointField field_;
  double offset;
};


class FieldReaderColor : public FieldReaderBase{

public:
  FieldReaderColor(const sensor_msgs::PointField& field, uint8_t* r, uint8_t* g, uint8_t* b){
    r_=r;
    g_=g;
    b_=b;
    field_ = field;
  }

  virtual void copy( int buffer_idx, int cloud_idx){
      // pack r/g/b into rgb
      uint32_t rgb = ((uint32_t)r_[buffer_idx] << 16 | (uint32_t)g_[buffer_idx] << 8 | (uint32_t)b_[buffer_idx]);
      *( (float *) ( odata_ +point_size_*cloud_idx +field_.offset   ) )  =  *reinterpret_cast<float*>(&rgb);
  }

protected:
  uint8_t * r_;
  uint8_t* g_;
  uint8_t* b_;
  sensor_msgs::PointField field_;
};



void readE57Header(StructureNode& scan,
                sensor_msgs::PointCloud2& cloud,
                Eigen::Vector4d& origin,
                Eigen::Quaterniond& rot){

  if(scan.isDefined("indexBounds")) {
   StructureNode indexBounds(scan.get("indexBounds"));
   if(indexBounds.isDefined("columnMaximum"))
       cloud.width = IntegerNode(indexBounds.get("columnMaximum")).value() -
                        IntegerNode(indexBounds.get("columnMinimum")).value() + 1;

       if(indexBounds.isDefined("rowMaximum"))
       cloud.height = IntegerNode(indexBounds.get("rowMaximum")).value() -
                 IntegerNode(indexBounds.get("rowMinimum")).value() + 1;
       cloud.is_dense=true;
     }
    else{
      CompressedVectorNode points(scan.get("points"));
      cloud.width = points.childCount();
      cloud.height =1;
    }


  // Get pose structure for scan.

    if(scan.isDefined("pose"))
    {
    StructureNode pose(scan.get("pose"));
    if(pose.isDefined("rotation"))
    {
            StructureNode rotation(pose.get("rotation"));
            rot = Eigen::Quaterniond(  FloatNode(rotation.get("w")).value(),
                                       FloatNode(rotation.get("x")).value(),
                                       FloatNode(rotation.get("y")).value(),
                                       FloatNode(rotation.get("z")).value());
    }
    if(pose.isDefined("translation"))
    {
      StructureNode translation(pose.get("translation"));
      origin[0]= FloatNode(translation.get("x")).value();
      origin[1] = FloatNode(translation.get("y")).value();
      origin[2] = FloatNode(translation.get("z")).value();
      origin[3] = 1;
    }
    }

}


/*
 * The majority of this code is adapted from the E57 reader example.
 */

pcl::E57Reader::E57Reader() : read_buffer_size_(100), scan_number_(0)
{
  pt_offset_ = Eigen::Vector4d::Zero();
}


pcl::E57Reader::~E57Reader()
{
}





int pcl::E57Reader::readHeader(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
                               Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
                               int & file_version, int & data_type, unsigned int & data_idx, const int offset)
{
  try {
         /// Read file from disk
         e57::ImageFile imf(file_name.c_str(), "r");
         StructureNode root = imf.root();

         /// Make sure vector of scans is defined and of expected type.
         /// If "/data3D" wasn't defined, the call to root.get below would raise an exception.
         if (!root.isDefined("/data3D")) {
             cerr << "File doesn't contain 3D images" << endl;
             return 0;
         }
         Node n = root.get("/data3D");
         if (n.type() != E57_VECTOR) {
             cerr << "bad file" << endl;
             return 0;
         }
         /// The node is a vector so we can safely get a VectorNode handle to it.
         /// If n was not a VectorNode, this would raise an exception.
         VectorNode data3D(n);

         StructureNode scan(data3D.get(scan_number_));

         Eigen::Vector4d origind;
         Eigen::Quaterniond quatd;
         readE57Header(scan,cloud,origind, quatd);
         origind += -pt_offset_;
         origin = origind.cast<float>();
         orientation = quatd.cast<float>();

         imf.close();
     } catch(E57Exception& ex) {
         ex.report(__FILE__, __LINE__, __FUNCTION__);
         return -1;
     } catch (std::exception& ex) {
         cerr << "Got an std::exception, what=" << ex.what() << endl;
         return -1;
     } catch (...) {
         cerr << "Got an unknown exception" << endl;
         return -1;
     }
     return 1;
}



pcl::E57Writer::E57Writer()
{
}

sensor_msgs::PointField newField(std::string name, pcl::uint32_t offset, pcl::uint8_t datatype, pcl::uint32_t count){
  sensor_msgs::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = count;
  return f;
}

int pcl::E57Reader::read(const std::string & file_name, sensor_msgs::PointCloud2 & cloud,
                         Eigen::Vector4f & origin, Eigen::Quaternionf & orientation, int & file_version,
                         const int offset)
{

  int total_points =0;

  e57::uint8_t* color_r = NULL, *color_g = NULL, *color_b=NULL;
  float* intensity = NULL;
  std::vector<FieldReaderBase *> fieldreaders;
  uint32_t indexRow[read_buffer_size_];
  uint32_t indexColumn[read_buffer_size_];

  try {
     /// Read file from disk
     ImageFile imf(file_name.c_str(), "r");
     StructureNode root = imf.root();

     /// Make sure vector of scans is defined and of expected type.
     /// If "/data3D" wasn't defined, the call to root.get below would raise an exception.
     if (!root.isDefined("/data3D")) {
       pcl::console::print_error("[E57Reader] File doesn't contain 3D images\n");
         return 0;
     }
     Node n = root.get("/data3D");
     if (n.type() != E57_VECTOR) {
       pcl::console::print_error("[E57Reader] bad file");
         return 0;
     }

     /// The node is a vector so we can safely get a VectorNode handle to it.
     /// If n was not a VectorNode, this would raise an exception.
     VectorNode data3D(n);

     /// Get the selected scan.
     StructureNode scan(data3D.get(scan_number_));

     /// Get "points" field in scan.  Should be a CompressedVectorNode.
     CompressedVectorNode points(scan.get("points"));

     Eigen::Vector4d origind;
     Eigen::Quaterniond quatd;
     readE57Header(scan,cloud,origind, quatd);
     origind += -pt_offset_;
     origin = origind.cast<float>();
     orientation = quatd.cast<float>();

     /// Need to figure out if has Cartesian or spherical coordinate system.
     /// Interrogate the CompressedVector's prototype of its records.
     StructureNode proto(points.prototype());


     if ( !(proto.isDefined("cartesianX") &&
         proto.isDefined("cartesianY") &&
         proto.isDefined("cartesianZ") )) {
       pcl::console::print_error("[E57Reader]  File has no XYZ data\n  The data may be in spherical coordiantes.\n");
       return -1;  //there is no XYZ data
     }

       vector<SourceDestBuffer> destBuffers;

       cloud.fields.push_back(newField("x",0,sensor_msgs::PointField::FLOAT32,1));
       cloud.fields.push_back(newField("y",4,sensor_msgs::PointField::FLOAT32,1));
       cloud.fields.push_back(newField("z",8,sensor_msgs::PointField::FLOAT32,1));


       double x[read_buffer_size_];     destBuffers.push_back(SourceDestBuffer(imf, "cartesianX", x, read_buffer_size_, true, true));
       double y[read_buffer_size_];     destBuffers.push_back(SourceDestBuffer(imf, "cartesianY", y, read_buffer_size_, true, true));
       double z[read_buffer_size_];     destBuffers.push_back(SourceDestBuffer(imf, "cartesianZ", z, read_buffer_size_, true, true));

       fieldreaders.push_back( new FieldReader<float, double>(cloud.fields[0], x));
       fieldreaders.push_back( new FieldReader<float, double>(cloud.fields[1], y));
       fieldreaders.push_back( new FieldReader<float, double>(cloud.fields[2], z));

       int field_offset = 12;
       if ( proto.isDefined("colorGreen") &&
           proto.isDefined("colorRed") &&
           proto.isDefined("colorBlue") ){
         color_r = new  uint8_t[read_buffer_size_];
         color_g = new  uint8_t[read_buffer_size_];
         color_b = new  uint8_t[read_buffer_size_];
         cloud.fields.push_back(newField("rgb",field_offset,sensor_msgs::PointField::FLOAT32,1));
         field_offset+=4;
         destBuffers.push_back(SourceDestBuffer(imf, "colorGreen", color_g, read_buffer_size_, true) );
         destBuffers.push_back(SourceDestBuffer(imf, "colorRed", color_r, read_buffer_size_, true) );
         destBuffers.push_back(SourceDestBuffer(imf, "colorBlue", color_b, read_buffer_size_, true) );

         fieldreaders.push_back( new FieldReaderColor(cloud.fields.back(), color_r, color_g, color_b));
       }

       if (proto.isDefined("intensity") ){
         intensity = new float[read_buffer_size_];
         destBuffers.push_back(SourceDestBuffer(imf, "intensity", intensity, read_buffer_size_, true, true) );
         cloud.fields.push_back(newField("intensity",field_offset,sensor_msgs::PointField::FLOAT32,1));
         field_offset+= 4;
         fieldreaders.push_back( new FieldReader<float, float>(cloud.fields.back(), intensity));
       }

       int point_size =0;
       int field_size[] = {1,1,2,2,4,4,4,8};
       for(int i=0; i< cloud.fields.size(); i++) point_size += cloud.fields[i].count* field_size[cloud.fields[i].datatype-1];

       total_points = cloud.width*cloud.height;
       cloud.data.resize( total_points*point_size );
       cloud.point_step = point_size;

       for(int i=0; i<fieldreaders.size(); i++) fieldreaders[i]->setup(cloud.data.data(), point_size);

       if (cloud.height > 1){
         destBuffers.push_back(SourceDestBuffer(imf, "rowIndex", indexRow, read_buffer_size_, true,true));
         destBuffers.push_back(SourceDestBuffer(imf, "columnIndex", indexColumn, read_buffer_size_, true,true));
       }

        /// Create a reader of the points CompressedVector, try to read first block of N points
        /// Each call to reader.read() will fill the xyz buffers until the points are exhausted.
        CompressedVectorReader reader = points.reader(destBuffers);
        int read_count =-1;
        read_count = reader.read();
        int cloud_index=0;
        while ( read_count > 0 ){
          for(int j=0; j<read_count; j++){
            if (cloud.height>1) cloud_index = (cloud.height-indexRow[j]-1) * cloud.width + indexColumn[j];
            else cloud_index ++;
            for(int i=0; i<fieldreaders.size(); i++){
              fieldreaders[i]->copy(j, cloud_index);
            }
          }
          read_count = reader.read();
        }
        imf.close();

     } catch(E57Exception& ex) {
       //clean up allocated memory
       if (color_b!= NULL){ delete color_b; delete color_g; delete color_r;}
       if (intensity != NULL ) delete intensity;
       for(int i=0; i< fieldreaders.size(); i++) delete fieldreaders[i];


       ex.report(__FILE__, __LINE__, __FUNCTION__);
       return -1;
     } catch (std::exception& ex) {
         cerr << "Got an std::exception, what=" << ex.what() << endl;

         //clean up allocated memory
         if (color_b!= NULL){ delete color_b; delete color_g; delete color_r;}
         if (intensity != NULL ) delete intensity;
         for(int i=0; i< fieldreaders.size(); i++) delete fieldreaders[i];

         return -1;
     } catch (...) {
         cerr << "Got an unknown exception" << endl;

         //clean up allocated memory
         if (color_b!= NULL){ delete color_b; delete color_g; delete color_r;}
         if (intensity != NULL ) delete intensity;
         for(int i=0; i< fieldreaders.size(); i++) delete fieldreaders[i];
         return -1;
     }

     if (color_b!= NULL){ delete color_b; delete color_g; delete color_r;}
     if (intensity != NULL ) delete intensity;
     for(int i=0; i< fieldreaders.size(); i++) delete fieldreaders[i];

 return total_points;
}



pcl::E57Writer::~E57Writer()
{
}




