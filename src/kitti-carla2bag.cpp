#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

static const int READ_SIZE = 16000;
static const int WRITE_SIZE = 16000;

class Point3D {

public:
    Eigen::Vector3d raw_pt;
    Eigen::Vector3d pt;
    double alpha_timestamp;
    double timestamp;
    int index_frame;
    int label;

    Point3D()
    {
        alpha_timestamp = 0.0;
        timestamp = 0.0;
        index_frame = -1;
        label = 0;
    }
};

bool time_list(Point3D &point_1, Point3D &point_2)
{
    return (point_1.alpha_timestamp < point_2.alpha_timestamp);
};

enum openMode{ fileOpenMode_OUT = 0, fileOpenMode_IN = 1 };

class File
{
public:
    File(string path, openMode flag);
    ~File();


protected:
    const openMode _mode;
    string     _path;
    fstream    _file;
};

File::File(string path, openMode flag) : _path(path), _mode(flag)
{
    switch (_mode)
    {
    case fileOpenMode_IN:
    {
        _file = fstream(_path.c_str(), ios::in | ios::binary);
        break;
    }
    case fileOpenMode_OUT:
    {
        _file = fstream(_path.c_str(), ios::out | ios::binary);
        break;
    }
    }

    if (!_file.good())
    {
        cout << "ERROR: can't open " << _path << endl;
    }
}


File::~File()
{
    _file.close();
}

enum plyFormat{ binary_little_endian = 0, binary_big_endian = 1, ascii = 2 };
enum plyTypes{ float32 = 0, float64 = 1, uchar2 = 2, int32 = 3, otherxx = -1 };


class PlyFile :
    public File
{
public:
    PlyFile(string path, openMode flag);
    ~PlyFile();
    
    void readFile(char*& points, int& pointSize, int& numPoints);

    void displayInfos();
    
    static const int READ_SIZE = 16000;
    static const int WRITE_SIZE = 16000;

private:
    void readHeader();

private:
    string    _header;
    plyFormat _format;

    int       _propertyNum;
    string*   _propertyName;
    plyTypes* _propertyType;
    int*      _propertySize;

    int   _numPoints;
    int   _pointSize;
};

PlyFile::PlyFile(string path, openMode flag) : File(path, flag), _header(""), _format(binary_little_endian), 
_propertyNum(0), _propertyType(NULL), _propertySize(NULL), _propertyName(NULL), 
_numPoints(0), _pointSize(0)
{
    if (_mode == fileOpenMode_IN)
    {
        readHeader();
    }
}


PlyFile::~PlyFile()
{
    delete[] _propertyType;
    delete[] _propertySize;
    delete[] _propertyName;
}



void PlyFile::readHeader()
{
    string tmpStr = "";
    
    do
    {
        getline(_file, tmpStr);
        _header += tmpStr + "\n";
    } while (tmpStr.find("end_header") != 0);

    stringstream streamHeader(_header);
    string strTmp = "";
    list<plyTypes> typePptTmp;
    list<int>      sizePptTmp;
    list<string>   namePptTmp;

    while (!streamHeader.eof())
    {
        streamHeader >> strTmp;
        
        if (strTmp.compare("format") == 0)
        {
            streamHeader >> strTmp;
            if (strTmp.compare("binary_little_endian") == 0)      _format = binary_little_endian;
            else if (strTmp.compare("binary_big_endian") == 0)    _format = binary_big_endian;
            else if (strTmp.compare("ascii") == 0)                _format = ascii;  
        }

        if (strTmp.compare("element") == 0)
        {
            streamHeader >> strTmp;
            if (strTmp.compare("vertex") == 0) streamHeader >> _numPoints;
        }

        if (strTmp.compare("property") == 0)
        {
            _propertyNum++;
            streamHeader >> strTmp;
            if ((strTmp.compare("float32") == 0) | (strTmp.compare("float") == 0))
            {
                typePptTmp.push_back(float32);
                sizePptTmp.push_back(4);
            }
            else if ((strTmp.compare("float64") == 0) | (strTmp.compare("double") == 0))
            {
                typePptTmp.push_back(float64);
                sizePptTmp.push_back(8);
            }
            else if ((strTmp.compare("int") == 0))
            {
                typePptTmp.push_back(int32);
                sizePptTmp.push_back(4);
            }
            else if ((strTmp.compare("uchar") == 0))
            {
                typePptTmp.push_back(uchar2);
                sizePptTmp.push_back(1);
            }
            else
            {
                typePptTmp.push_back(otherxx);
                sizePptTmp.push_back(4); // Default
            }

            streamHeader >> strTmp;
            namePptTmp.push_back(strTmp);
        }
    }
    
    _propertyType = new plyTypes[_propertyNum];
    _propertySize = new int[_propertyNum];
    _propertyName = new string[_propertyNum];

    for (int i(0); i < _propertyNum; i++)
    {
        _propertyType[i] = typePptTmp.front();
        _propertySize[i] = sizePptTmp.front();
        _propertyName[i] = namePptTmp.front();
        typePptTmp.pop_front();
        sizePptTmp.pop_front();
        namePptTmp.pop_front();

        _pointSize += _propertySize[i];
    }
}

void PlyFile::readFile(char*& points, int& pointSize, int& numPoints)
{
    switch (_format)
    {
    case binary_little_endian:
    {
        if (points != 0)
        {
            delete[] points;
        }

        points = new char[(unsigned long long int)_pointSize*(unsigned long long int)_numPoints];
        unsigned long long int bufferSize = (unsigned long long int)_pointSize*(unsigned long long int)_numPoints;

        unsigned long long int n = bufferSize / (unsigned long long int)READ_SIZE;
        unsigned long long int r = bufferSize % (unsigned long long int)READ_SIZE;

        for (unsigned long long int i(0); i < n; i++)
        {
            _file.read(points + i*(unsigned long long int)READ_SIZE, READ_SIZE);
        }

        _file.read(points + n*(unsigned long long int)READ_SIZE, r);

        numPoints = _numPoints;
        pointSize = _pointSize;

        break;
    }
    case binary_big_endian:
    {
        cout << "WARNING: function not implemented for binary big endian file" << endl;
        break;
    }
    case ascii:
    {
        cout << "WARNING: function not implemented for ascii file" << endl;
        break;
    }
    }   
}

void PlyFile::displayInfos()
{
    cout << "------------------------------------------------------" << endl;
    cout << " PLY File : " << _path << endl;
    cout << "------------------------------------------------------" << endl;
    cout << "  - format     : " << _format << endl;
    cout << "  - num points : " << _numPoints << endl;
    cout << "  - properties : " << endl;
    for (int i(0); i < _propertyNum; i++)
    {
        cout << "     - " << _propertyName[i] << " :    " << _propertyType[i] << " |    " << _propertySize[i] << " bytes " << endl;
    }
    cout << "------------------------------------------------------" << endl << endl;
}

std::vector<Point3D> read_kitti_carla_pointcloud(const std::string &path) {
    std::vector<Point3D> frame;

    PlyFile plyFileIn(path, fileOpenMode_IN);
    char *dataIn = nullptr;
    int sizeOfPointsIn = 0;
    int numPointsIn = 0;
    plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);
    frame.reserve(numPointsIn);

    double frame_last_timestamp = 0.0;
    double frame_first_timestamp = 1000000000.0;
    for (int i(0); i < numPointsIn; i++) {

        unsigned long long int offset =
                (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
        Point3D new_point;
        new_point.raw_pt[0] = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.raw_pt[1] = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.raw_pt[2] = *((float *) (dataIn + offset));
        offset += sizeof(float);


        new_point.pt = new_point.raw_pt;
        double cos = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.timestamp = *((float *) (dataIn + offset));
        offset += sizeof(float);
        uint32_t index = *((uint32_t *) (dataIn + offset));
        offset += sizeof(uint32_t);
        new_point.label = *((uint32_t *) (dataIn + offset));
        offset += sizeof(uint32_t);

        if (new_point.timestamp < frame_first_timestamp) {
            frame_first_timestamp = new_point.timestamp;
        }

        if (new_point.timestamp > frame_last_timestamp) {
            frame_last_timestamp = new_point.timestamp;
        }

        double r = new_point.raw_pt.norm();
        if ((r > 0.5) && (r < 100.0))
            frame.push_back(new_point);
    }

    for (int i(0); i < (int) frame.size(); i++) {
        frame[i].alpha_timestamp = min(1.0, max(0.0, 1 - (frame_last_timestamp - frame[i].timestamp) /
                                                         (frame_last_timestamp - frame_first_timestamp)));
    }
    frame.shrink_to_fit();

    delete[] dataIn;
    return frame;
}

void loadTimestamp(const std::string &path, std::vector<int> &vIndex, std::vector<double> &vTime)
{
    std::ifstream infile;
    infile.open(path);

    while(!infile.eof())
    {
        std::string s;
        getline(infile, s);

        if(s.length() == 0)
            break;

        int index;
        double timestamp;

        std::stringstream ss;
        ss << s;
        ss >> index;
        ss >> timestamp;

        vIndex.push_back(index);
        vTime.push_back(timestamp);
    }

    infile.close();

    std::cout << "load timestamp successfully!" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti-carla2bag");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    std::string time_file_path = dataset_folder + sequence_number + "/full_ts_camera.txt";
    std::vector<int> vIndex;
    std::vector<double> vTime;
    loadTimestamp(time_file_path, vIndex, vTime);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    ros::Rate r(10.0 / publish_delay);

    for (int i = 0; i < vIndex.size(); i++)
    {
        double timestamp = vTime[i];

        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder + sequence_number + "/images_rgb/" << std::setfill('0') << std::setw(4) << vIndex[i] << "_0.png";
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
        assert(!left_image.empty());

        // read lidar point cloud
        std::stringstream lidar_data_path; 
        lidar_data_path << dataset_folder + sequence_number + "/correct/frame_" << std::setfill('0') << std::setw(4) << vIndex[i] << ".ply";
        std::vector<Point3D> lidar_data = read_kitti_carla_pointcloud(lidar_data_path.str());

        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (int j = 0; j < lidar_data.size(); j++)
        {
            pcl::PointXYZI point;
            point.x = lidar_data[j].raw_pt(0, 0);
            point.y = lidar_data[j].raw_pt(1, 0);
            point.z = lidar_data[j].raw_pt(2, 0);
            point.intensity = lidar_data[j].label;
            laser_cloud.push_back(point);
        }

        assert(laser_cloud.points.size() > 0);

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = "/camera_init";

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();

        if (to_bag)
        {
            std::cout << std::fixed << "timestamp = " << timestamp << " index = " << vIndex[i] << std::endl;
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/kitti/velo/pointcloud", ros::Time::now(), laser_cloud_msg);
        }

        r.sleep();
    }
    
    bag_out.close();
    std::cout << "Done \n";

    return 0;
}
