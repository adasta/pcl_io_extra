
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
  FieldReaderColor(const pcl::PCLPointField& field, uint8_t* r, uint8_t* g, uint8_t* b){
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
                pcl::toPCLPointCloud2& cloud,
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




sensor_msgs::PointField newField(std::string name, pcl::uint32_t offset, pcl::uint8_t datatype, pcl::uint32_t count){
  pcl::PCLPointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = count;
  return f;
}

int pcl::E57Reader::read(const std::string & file_name, pcl::toPCLPointCloud2 & cloud,
                         Eigen::Vector4f & origin, Eigen::Quaternionf & orientation, int & file_version,
                         const int offset)
{
  assert(false && "unimplemented");
  return -1;
}

