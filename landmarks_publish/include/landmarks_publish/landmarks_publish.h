#ifndef BASE_PERCEPTION_LANDMARKS_PUBLISH_H_
#define BASE_PERCEPTION_LANDMARKS_PUBLISH_H_

#include "iostream"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "functional"
#include "utility"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
namespace landmarks_publish
{
class LandmarksPublish
{
public:
   LandmarksPublish(ros::NodeHandle& nh);
   ~LandmarksPublish();
private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_scan_sub_;
  ros::Publisher landmarks_pub_;
  ros::Timer publish_timer_;
  void publishLandmarksTimer(const ros::TimerEvent& event);
  void laserScanCb(const sensor_msgs::LaserScanConstPtr& msg);
  std::string laser_topic_name_;
  std::string landmark_topic_name_;
  std::vector<int> detectReflectors(std::vector<double> intensities);
  double landmarks_publish_period_;
  double translation_weight_;
  double rotation_weight_;
  double intensity_upper_;
  double intensity_lower_;
  std::string tracking_frame_;
  int detect_id_mode_;
  std::vector<geometry_msgs::PoseStamped> reflectors_pose_;
  std::vector<std::string> reflectors_id_;
  std::vector<geometry_msgs::PoseStamped> computeReflectorPose(std::vector<double> ranges,
                                                        std::vector<int> reflectors);
  std::vector<std::string> computeReflectorId(std::vector<geometry_msgs::PoseStamped> pose);
  void generateLandmarksData(cartographer_ros_msgs::LandmarkList& landmark_list);
  bool is_find_reflector_;
  double laser_angle_min_;
  double laser_angle_increment_;
  std::string laser_frame_;
  tf::TransformListener* listener_;
  bool transformPose(std::string target_frame,
                     geometry_msgs::PoseStamped& input,
                     geometry_msgs::PoseStamped& output);
  int64_t reflector_id_;
  //维护反光柱子地图
  std::map<std::string, geometry_msgs::PoseStamped> reflector_map_;
  double close_tolerance_;
  //ros::Time last_laser_time_;
  visualization_msgs::Marker reflectors_mark_;
  visualization_msgs::Marker reflectors_mark_map_;
  visualization_msgs::Marker reflectors_text_mark_;
  visualization_msgs::Marker reflectors_text_mark_map_;
  //visualization_msgs::MarkerArray reflectors_text_array_mark_;
  //visualization_msgs::MarkerArray reflectors_text_array_mark_map_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_map_pub_;
  ros::Publisher marker_text_pub_;
  ros::Publisher marker_map_text_pub_;
  void initMarker();
  ros::Publisher reflector_map_size_pub_;
  //2021-05-08
  int detect_frame_count_;
  std::map<std::string, int> reflector_id_count_;
  bool is_pub_empty_;
  std::string publish_mode_;
  ros::Subscriber change_pub_mode_sub_;
  void changePubModeSubCb(const std_msgs::StringConstPtr& msg);
};
}

#endif