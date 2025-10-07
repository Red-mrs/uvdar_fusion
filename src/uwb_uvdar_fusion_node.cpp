/* 
* This file is to fuse data from uwb module and uvdar camera
* Works with 4 modules with fixed ids
* 0xAA - 28
* 0xBB - 29
* 0xCC - 30
* 0xDD - 31
*/

/* TODO:
 * filter the data (particle filter?)
 * fuse uwb_data with uvdar_data based on point id    DONE
 * transform data from camera frame to drone frame    DONE
 * use camera calibration file                        DONE
*/

#include <map>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/String.h>
#include <fstream>
#include "uwb_uvdar_fusion/UwbRangeStamped.h"
#include "uwb_uvdar_fusion/ImagePointsWithFloatStamped.h"
#include "uwb_uvdar_fusion/Point2DWithFloat.h"
#include "uwb_uvdar_fusion/UwbUvdarResult.h"
#include "OCamCalib/ocam_functions.h"
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <boost/filesystem/operations.hpp>
#include <mrs_lib/param_loader.h>

#define deg2rad(X) ((X)*0.01745329251)
#define rad2deg(X) ((X)*57.2957795131)
#define sqr(X)     ((X)*(X))

#define write_data false

namespace e = Eigen;

bool debug_mode;
std::string uvdar_topic;
const int rate = 50; //rate in Hz
const std::array<int, 4> ids = {28, 29, 30, 31};
int image_index = 0;

std::vector<struct ocam_model> _oc_models_;
std::vector<e::Quaterniond> _center_fix_;
std::vector<std::string> _calib_files_;

#if write_data
std::ofstream outfile;
bool header_written = false;
#endif

e::Vector3d directionFromCamPoint(cv::Point2d point, int image_index);

ros::Publisher uwb_uvdar_fuser_raw_data;
uwb_uvdar_fusion::UwbUvdarResult result_data;

struct detected_entry {
  uint32_t detected_adress;
  float distance;
};

struct uwb_data{
  ros::Time  time_stamp;
  uint32_t own_address;
  std::map<uint32_t, float> detected;
} uwb_data;

struct point_with_float{
  double x;
  double y;
  double id;
};

struct uvdar_data{
  ros::Time time_stamp;
  std::vector<point_with_float> points;
} uvdar_data;

std::ostream& operator<<(std::ostream& os, const std::map<int, std::pair<double,double>>& averages) {
    os << "Averages:\n";
    for (const auto& kv : averages) {
        int id = kv.first;
        double x = kv.second.first;
        double y = kv.second.second;
        os << "  ID=" << id
           << " x=" << x
           << " y=" << y
           << '\n';
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, struct uwb_data& d) {
  os << "\nstamp=" << d.time_stamp.sec << "." << d.time_stamp.nsec
     << " \nown=" << d.own_address
     << " detected list:\n";
  for (const auto& v : d.detected) {
    os << " addr=0x" << std::hex << v.first
       << " distance=" << v.second << '\n';
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const point_with_float& pt) {
    os << "(x=" << pt.x << ", y=" << pt.y << ", id=" << pt.id << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, struct uvdar_data& d) {
    os << "Time: " << d.time_stamp.sec << "." << d.time_stamp.nsec << "\n";
    os << "Points:\n";
    for (size_t i = 0; i < d.points.size(); ++i) {
        os << "  [" << i << "] " << d.points[i] << "\n";
    }
    return os;
}

bool loadCalibrations(){
        std::string file_name;
        if (_calib_files_.empty()) {
            ROS_WARN("[UVDARPoseCalculator]: _calib_files_ is empty, using default calibration.");
            _calib_files_.push_back("default");
        }
          _oc_models_.resize(_calib_files_.size());
        int i=0;
        for (auto calib_file : _calib_files_){
          if (calib_file == "default"){
            file_name = ros::package::getPath("uwb_uvdar_fusion")+"/config/ocamcalib/calib_results_bf_uv_fe.txt";
          }
          else {
            file_name = calib_file;
          }

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Loading camera calibration file [" << file_name << "]");
          if (!(boost::filesystem::exists(file_name))){
            ROS_ERROR_STREAM("[UVDARPoseCalculator Calibration file [" << file_name << "] does not exist!");
            return false;
          }

          get_ocam_model(&_oc_models_.at(i), (char*)(file_name.c_str()));
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Calibration parameters for virtual camera " << i << " came from the file " <<  file_name);
          for (int j=0; j<_oc_models_.at(i).length_pol; j++){
            if (isnan(_oc_models_.at(i).pol[j])){
              ROS_ERROR("[UVDARPoseCalculator]: Calibration polynomial containts NaNs! Returning.");
              return false;
            }
          }

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Camera resolution  is: " << _oc_models_.at(i).width << "x" << _oc_models_.at(i).height);

          /* auto center_dir = directionFromCamPoint(cv::Point3d(_oc_models_[i].yc,_oc_models_[i].xc,0),i); */
          auto center_dir = directionFromCamPoint(cv::Point2d(_oc_models_.at(i).width/2.0,_oc_models_.at(i).height/2.0),i);
          auto zero_dir = e::Vector3d(0,0,1);

          double x_ang = acos(e::Vector3d(center_dir.x(),0,center_dir.z()).normalized().dot(zero_dir));
          if (center_dir.x()<0){
            x_ang = -x_ang;
          }

          double y_ang = acos(e::Vector3d(0,center_dir.y(),center_dir.z()).normalized().dot(zero_dir));
          if (center_dir.y()<0){
            y_ang = -y_ang;
          }

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Central direction is: " << center_dir.transpose());
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Offset in angles from central direction: X=" << rad2deg(x_ang) << ": Y=" << rad2deg(y_ang) );

          _center_fix_.push_back(e::Quaterniond(e::AngleAxisd(-x_ang*0.5, e::Vector3d(0,-1,0))*e::AngleAxisd(-y_ang*0.5, e::Vector3d(1,0,0))));

          i++;
        }
        return true;
      }

e::Vector3d directionFromCamPoint(cv::Point2d point, int image_index){
  double v_i[2] = {(double)(point.y), (double)(point.x)};
  double v_w_raw[3];
  cam2world(v_w_raw, v_i, &(_oc_models_[image_index]));
  return e::Vector3d(v_w_raw[1], v_w_raw[0], -v_w_raw[2]);
}      

// callback for distance
void distanceCallback(const uwb_uvdar_fusion::UwbRangeStamped::ConstPtr& msg)
{
  if (msg->range.own_address == msg->range.responder_address)
  {
    uwb_data.time_stamp = msg->header.stamp;
    uwb_data.detected[msg->range.initiator_address] = msg->range.distance;
    uwb_data.own_address = msg->range.own_address;
  }
  else if(msg->range.own_address == msg->range.initiator_address)
  {
    uwb_data.time_stamp = msg->header.stamp;
    uwb_data.detected[msg->range.responder_address] = msg->range.distance;
    uwb_data.own_address = msg->range.own_address;
  }

  if(debug_mode){
    ROS_INFO_STREAM(uwb_data);
  }
}

// callback for blinkers
void blinkersCallback(const uwb_uvdar_fusion::ImagePointsWithFloatStamped::ConstPtr& msg)
{
  uvdar_data.points.clear();

  uvdar_data.time_stamp = msg->stamp;

  for (const auto& p : msg->points){
    point_with_float tmp;
    tmp.x = p.x;
    tmp.y = p.y;
    tmp.id = p.value; 
    uvdar_data.points.push_back(tmp);
  }
    if(debug_mode){
      ROS_INFO_STREAM(uvdar_data);
    }
}

std::map<int, std::pair<double,double>> computeAverages(const std::vector<point_with_float>& points) {
    std::map<int, double> sum_x;
    std::map<int, double> sum_y;
    std::map<int, int> count;

    for (const auto& p : points) {
        int id = static_cast<int>(p.id);
        if (std::find(ids.begin(), ids.end(), id) != ids.end()) {
            sum_x[id] += p.x;
            sum_y[id] += p.y;
            count[id]  += 1;
        }
    }

    std::map<int, std::pair<double,double>> averages;
    for (auto id : ids) {
        if (count[id] > 0) {
            averages[id] = {
                sum_x[id] / count[id],
                sum_y[id] / count[id]
            };
        }
    }

    if (debug_mode){
      ROS_INFO_STREAM(averages);
    }

    return averages;
}

void publishFusionRaw(const ros::TimerEvent&)
{
  auto averages = computeAverages(uvdar_data.points);
  std::map<int,int> uwb2uvdar = {
      {0xAA, 28},
      {0xBB, 29},
      {0xCC, 30},
      {0xDD, 31}
  };

  for (const auto& det : uwb_data.detected) {
    int uwb_id = det.first;    
    float distance = det.second;

    auto it = uwb2uvdar.find(uwb_id);
    if (it != uwb2uvdar.end()) {
      int uvdar_id = it->second;

      result_data.header.stamp = ros::Time::now();
      result_data.own_id = uwb_data.own_address;
      result_data.id = uvdar_id;
      result_data.distance = distance;

      if (averages.count(uvdar_id)) {
        cv::Point2d point{averages[uvdar_id].first,averages[uvdar_id].second};
        e::Vector3d vec = directionFromCamPoint(point,image_index); 
        result_data.x = vec[0]*distance;
        result_data.y = vec[1]*distance;
        result_data.z = sqrt(sqr(distance)-sqr(vec[0]*distance)-sqr(vec[1]*distance));
      }
      if ((result_data.x || result_data.y || result_data.z) != 0)
      {
      uwb_uvdar_fuser_raw_data.publish(result_data);
#if write_data
      if (!header_written) {
        outfile << "seq,stamp,id,own_id,x,y,z,distance" << std::endl; 
        header_written = true;
      }
      outfile << result_data.header.seq << ","
              << result_data.header.stamp.sec << "." << result_data.header.stamp.nsec << ","
              << result_data.id << ","
              << result_data.own_id << ","
              << result_data.x << ","
              << result_data.y << ","
              << result_data.z << ","
              << result_data.distance << ","
              << std::endl;
#endif
      }
      result_data = {}; 
   }
  }
}

//-------------------MAIN--------------------//

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "uwb_uvdar_fusion");
    ros::NodeHandle nh("~");
    
    nh.param("debug_mode", debug_mode, false);  
    nh.param<std::string>("uvdar_topic", uvdar_topic, "blinkers_seen_left");

    mrs_lib::ParamLoader param_loader(nh, "uwb_uvdar_fusion");
    param_loader.loadParam("calib_files", _calib_files_, _calib_files_);
    if (_calib_files_.empty()) {
      ROS_ERROR("[UVDARPoseCalculator]: No camera calibration files were supplied. You can even use \"default\" for the cameras, but no calibration is not permissible. Returning.");
      ros::shutdown();
    }

    if (!loadCalibrations()){
      ROS_ERROR("[UVDARPoseCalculator The camera calibration files could not be loaded!");
      ros::shutdown();
    }
    
#if write_data
    std::string path = ros::package::getPath("uwb_uvdar_fusion");
    std::string filepath = path + "/output.csv";

    outfile.open(filepath, std::ios::out | std::ios::app);
    if (!outfile.is_open()){
      ROS_ERROR("Could not open file at %s", filepath.c_str());
      return -1;
    }
#endif

    uwb_uvdar_fuser_raw_data = nh.advertise<uwb_uvdar_fusion::UwbUvdarResult >("uwb_uvdar_result_raw", 10);

    ros::Subscriber distance_sub = nh.subscribe("/uav/uvdar_driver/distance", 10, distanceCallback);

    ros::Subscriber blinkers_sub = nh.subscribe(std::string("/uav2/uvdar/") + uvdar_topic, 10, blinkersCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0/rate), publishFusionRaw);

    ROS_INFO("UWB_UVdar fusion node started...");

    ros::spin();

    return 0;
}

