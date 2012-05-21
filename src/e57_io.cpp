
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
  virtual void copy(uint64_t data_offset, int copy_n) =0;

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
  }
  FieldReader(const sensor_msgs::PointField& field, BUFFERT* buffer){
    field_ = field;
    buffer_ = buffer;
  }
 virtual  ~FieldReader(){}
  virtual void copy(uint64_t data_offset, int copy_n){
    for(uint64_t i=0; i< copy_n; i++){
      *( (PCLT *) ( odata_ +point_size_*i +field_.offset  +data_offset  ) )  =  buffer_[i];
    }
  }

  void setup( uint8_t* output, int point_size){
    odata_ = output;
    point_size_ = point_size;
  }

private:
  BUFFERT* buffer_;
  sensor_msgs::PointField field_;
};


class FieldReaderColor : public FieldReaderBase{

public:
  FieldReaderColor(const sensor_msgs::PointField& field, uint8_t* r, uint8_t* g, uint8_t* b){
    r_=r;
    g_=g;
    b_=b;
    field_ = field;
  }

  virtual void copy(uint64_t data_offset, int copy_n){
    for(int i=0; i< copy_n; i++){
      // pack r/g/b into rgb
      uint32_t rgb = ((uint32_t)r_[i] << 16 | (uint32_t)g_[i] << 8 | (uint32_t)b_[i]);
      *( (float *) ( odata_ +point_size_*i +field_.offset + data_offset   ) )  =  *reinterpret_cast<float*>(&rgb);
    }
  }

protected:
  uint8_t * r_;
  uint8_t* g_;
  uint8_t* b_;
  sensor_msgs::PointField field_;
};


/*
 * The majority of this code is adapted from the E57 reader example.
 */

pcl::E57Reader::E57Reader() : read_buffer_size_(100), scan_number_(0)
{

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
             cout << "File doesn't contain 3D images" << endl;
             return 0;
         }
         Node n = root.get("/data3D");
         if (n.type() != E57_VECTOR) {
             cout << "bad file" << endl;
             return 0;
         }

         //TODO Implement header reading

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


  try {
     /// Read file from disk
     ImageFile imf(file_name.c_str(), "r");
     StructureNode root = imf.root();

     /// Make sure vector of scans is defined and of expected type.
     /// If "/data3D" wasn't defined, the call to root.get below would raise an exception.
     if (!root.isDefined("/data3D")) {
         cout << "File doesn't contain 3D images" << endl;
         return 0;
     }
     Node n = root.get("/data3D");
     if (n.type() != E57_VECTOR) {
         cout << "bad file" << endl;
         return 0;
     }

     /// The node is a vector so we can safely get a VectorNode handle to it.
     /// If n was not a VectorNode, this would raise an exception.
     VectorNode data3D(n);

     /// Print number of children of data3D.  This is the number of scans in file.
     int64_t scanCount = data3D.childCount();
     cout << "Number of scans in file:" << scanCount << endl;

     /// Get the selected scan.
     StructureNode scan(data3D.get(scan_number_));
     cout << "got:" << scan.pathName() << endl;

     /// Get "points" field in scan.  Should be a CompressedVectorNode.
     CompressedVectorNode points(scan.get("points"));
     cout << "got:" << points.pathName() << endl;

     total_points =  points.childCount();
     cloud.width = total_points;
     cloud.height =1;
     /// Need to figure out if has Cartesian or spherical coordinate system.
     /// Interrogate the CompressedVector's prototype of its records.
     StructureNode proto(points.prototype());


     if ( !(proto.isDefined("cartesianX") &&
         proto.isDefined("cartesianY") &&
         proto.isDefined("cartesianZ") )) {
       std::cout << "File has no XYZ data\n  The data may be in spherical coordiantes.\n";
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
         cloud.fields.push_back(newField("rgba",field_offset,sensor_msgs::PointField::FLOAT32,1));
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

       cloud.data.resize( total_points*point_size );

       for(int i=0; i<fieldreaders.size(); i++) fieldreaders[i]->setup(cloud.data.data(), point_size);

        /// Create a reader of the points CompressedVector, try to read first block of N points
        /// Each call to reader.read() will fill the xyz buffers until the points are exhausted.
        CompressedVectorReader reader = points.reader(destBuffers);
        int read_count =0;
        uint64_t read_offset=0;
        read_count = reader.read();
        while ( read_count > 0 ){
          for(int i=0; i<fieldreaders.size(); i++) fieldreaders[i]->copy(read_offset, read_count);
          read_offset = point_size*read_count+read_offset;
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




