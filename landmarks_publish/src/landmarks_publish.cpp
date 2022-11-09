#include "landmarks_publish/landmarks_publish.h"

using namespace landmarks_publish;

LandmarksPublish::LandmarksPublish(ros::NodeHandle& nh):
  nh_(nh), is_find_reflector_(false), reflector_id_(0)
{
  ROS_INFO("Construct LandmarksPublish Object!");
  nh_.param("tracking_frame", tracking_frame_, std::string("/base_link"));
  nh_.param("laser_topic_name", laser_topic_name_, std::string("scan_1"));
  nh_.param("landmark_topic_name", landmark_topic_name_, std::string("landmark"));
  nh_.param("landmark_publish_period", landmarks_publish_period_, 5.0);
  nh_.param("translation_weight", translation_weight_, 10.0);
  nh_.param("rotation_weight", rotation_weight_, 10.0);
  nh_.param("intensity_upper", intensity_upper_, 2000.0);
  nh_.param("intensity_lower", intensity_lower_, 1000.0);
  nh_.param("detect_id_mode", detect_id_mode_, 0);
  nh_.param("close_tolerance", close_tolerance_, 0.05);
  nh_.param("detect_frame_count", detect_frame_count_, 3);
  nh_.param("is_pub_empty", is_pub_empty_, false);
  nh_.param("publish_mode", publish_mode_, std::string("local"));
  ROS_INFO("intensity upper:%f lower:%f", intensity_upper_, intensity_lower_);
  laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic_name_, 1, &LandmarksPublish::laserScanCb, this);
  landmarks_pub_ = nh_.advertise<cartographer_ros_msgs::LandmarkList>(landmark_topic_name_, 1);
  publish_timer_ = nh_.createTimer(ros::Rate(landmarks_publish_period_), &LandmarksPublish::publishLandmarksTimer, this);
  listener_ = new tf::TransformListener(nh, ros::Duration(5.0), true);
  //listener_ = new tf::TransformListener(nh, ros::Duration(5.0), true);
  //listener_->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(30.0));
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("reflectors", 10);
  marker_map_pub_ = nh_.advertise<visualization_msgs::Marker>("reflectors_map", 10);
  marker_text_pub_ = nh_.advertise<visualization_msgs::Marker>("reflector_text", 10);
  marker_map_text_pub_ = nh_.advertise<visualization_msgs::Marker>("reflector_map_text", 10);
  reflector_map_size_pub_ = nh_.advertise<std_msgs::Int8>("reflector_map_size", 10);
  change_pub_mode_sub_ = nh_.subscribe<std_msgs::String>("change_landmark_pub_mode", 1, &LandmarksPublish::changePubModeSubCb, this);
  initMarker();
  ROS_INFO("Construct LandmarksPublish Object Finish!");
}

LandmarksPublish::~LandmarksPublish()
{}

void LandmarksPublish::initMarker()
{
  reflectors_mark_.header.frame_id = tracking_frame_;
  reflectors_mark_.ns = "Markers";
  reflectors_mark_.action = visualization_msgs::Marker::ADD;
  reflectors_mark_.pose.orientation.w = 1.0;
  reflectors_mark_.id = 0;
  

  reflectors_mark_.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  reflectors_mark_.scale.x = 0.2;
  reflectors_mark_.scale.y = 0.2;

  // Points are green
  reflectors_mark_.color.r = 1.0f;
  reflectors_mark_.color.a = 1.0;
  reflectors_mark_.lifetime = ros::Duration(1.0); //持续1s
  /**********************************************/
  reflectors_mark_map_.header.frame_id = "map";
  reflectors_mark_map_.ns = "Markers";
  reflectors_mark_map_.action = visualization_msgs::Marker::ADD;
  reflectors_mark_map_.pose.orientation.w = 1.0;
  reflectors_mark_map_.id = 0;
  

  reflectors_mark_map_.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  reflectors_mark_map_.scale.x = 0.1;
  reflectors_mark_map_.scale.y = 0.1;

  // Points are green
  reflectors_mark_map_.color.b = 1.0f;
  reflectors_mark_map_.color.a = 1.0;
  /**********************************************/
  reflectors_text_mark_.header.frame_id = tracking_frame_;
  reflectors_text_mark_.ns = "Markers";
  reflectors_text_mark_.action = visualization_msgs::Marker::ADD;
  reflectors_text_mark_.pose.orientation.w = 1.0;
  reflectors_text_mark_.id = 0;
  

  reflectors_text_mark_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // POINTS markers use x and y scale for width/height respectively
  // reflectors_text_mark_.scale.x = 0.2;
  // reflectors_text_mark_.scale.y = 0.2;
  reflectors_text_mark_.scale.z = 0.2;
  // Points are green
  reflectors_text_mark_.color.r = 1.0f;
  reflectors_text_mark_.color.a = 1.0;
  //reflectors_text_mark_.lifetime = ros::Duration(1.0); //持续1s
  /**************************************************/
  reflectors_text_mark_map_.header.frame_id = "map";
  reflectors_text_mark_map_.ns = "Markers";
  reflectors_text_mark_map_.action = visualization_msgs::Marker::ADD;
  reflectors_text_mark_map_.pose.orientation.w = 1.0;
  reflectors_text_mark_map_.id = 0;
  

  reflectors_text_mark_map_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // POINTS markers use x and y scale for width/height respectively
  // reflectors_text_mark_map_.scale.x = 0.2;
  // reflectors_text_mark_map_.scale.y = 0.2;
  reflectors_text_mark_map_.scale.z = 0.2;
  // Points are green
  reflectors_text_mark_map_.color.b = 1.0f;
  reflectors_text_mark_map_.color.a = 1.0;
  
}

void LandmarksPublish::changePubModeSubCb(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO("curr_pub_mode:%s change to:%s", publish_mode_.c_str(), msg->data.c_str());
  publish_mode_ = msg->data;
  if (publish_mode_ == "initial_local")
    detect_id_mode_ = 0;
  else 
    detect_id_mode_ = 1;
}

void LandmarksPublish::laserScanCb(const sensor_msgs::LaserScanConstPtr& msg)
{
  // if ((ros::Time::now() - last_laser_time_).toSec() < 0.15)
  //   return;
  laser_angle_min_ = msg->angle_min;
  laser_angle_increment_ = msg->angle_increment;
  laser_frame_ = msg->header.frame_id;
  std::vector<double> ranges, intensities;
  for (int i = 0; i < msg->intensities.size(); ++i)
  {
   ranges.push_back(msg->ranges[i]);
   intensities.push_back(msg->intensities[i]);
  }
  //当前帧识别到的reflectors
  std::vector<int> reflectors = detectReflectors(intensities);
  if (reflectors.empty()) 
  {
    is_find_reflector_ = false;
    reflectors_pose_.clear();
    reflectors_id_.clear();
    return;
  }
  reflectors_pose_ = computeReflectorPose(ranges, reflectors);
  reflectors_id_ = computeReflectorId(reflectors_pose_);
  if (!reflectors_id_.empty())
    is_find_reflector_ = true;
  //last_laser_time_ = ros::Time::now();
}

void LandmarksPublish::publishLandmarksTimer(const ros::TimerEvent& event)
{
  cartographer_ros_msgs::LandmarkList landmark_list;
  if (is_find_reflector_)
  {
    generateLandmarksData(landmark_list);
    landmarks_pub_.publish(landmark_list);
    is_find_reflector_ = false;
    marker_pub_.publish(reflectors_mark_);
    //marker_text_pub_.publish(reflectors_text_array_mark_);
    // if (reflectors_pose_.size() != reflectors_id_.size())
    // {
    //   ROS_ERROR("reflector'pose size mismatch reflector's id size.");
    //   return;
    // }
    if (reflectors_pose_.size() != reflectors_id_.size())
      return;
    int id = 0;
    for (int i = 0; i < reflectors_pose_.size(); ++i)
    {
      reflectors_text_mark_.pose = reflectors_pose_[i].pose;
      reflectors_text_mark_.text = reflectors_id_[i];
      reflectors_text_mark_.id = ++id;
      marker_text_pub_.publish(reflectors_text_mark_);
    }
  }
  else
  {
    landmark_list.header.stamp = ros::Time::now();
    landmark_list.header.frame_id = tracking_frame_;
    landmarks_pub_.publish(landmark_list);
  }
  
 
  //publish reflector map
  marker_map_pub_.publish(reflectors_mark_map_);
  //marker_map_text_pub_.publish(reflectors_text_array_mark_map_);
  int id = 0;
  for (auto itr : reflector_map_)
  {
    reflectors_text_mark_map_.pose = itr.second.pose;
    reflectors_text_mark_map_.text = itr.first;
    reflectors_text_mark_map_.id = ++id;
    marker_map_text_pub_.publish(reflectors_text_mark_map_);
  }
  //ROS_WARN_THROTTLE(10.0, "reflector map size:%d", reflectors_mark_map_.points.size());
  std_msgs::Int8 msg;
  msg.data = reflectors_mark_map_.points.size();
  reflector_map_size_pub_.publish(msg);
  //reflectors_mark_map_.points.clear();
  reflectors_mark_.points.clear();
  // generateLandmarksData(landmark_list);
  // landmarks_pub_.publish(landmark_list);
  // if (is_find_reflector_)
  // is_find_reflector_ = false;
}

std::vector<int> LandmarksPublish::detectReflectors(std::vector<double> intensities)
{
  std::vector<std::vector<std::pair<int, double>>> reflectors;
  std::vector<std::pair<int, double>> bucket;
  bool is_find = false;
  for (int i = 0; i < intensities.size(); ++i)
  {
    if (intensities[i] > intensity_lower_ && intensities[i] < intensity_upper_)
      //template<class T1, class T2> make_pair(T1&&, T2&&)传入2个右值引用，这里的i是左值
      //自动推断，完美转发机制？
    {
      bucket.push_back(std::make_pair(i, intensities[i]));
      is_find = true;
    } 
    // if (i == intensities.size())
    // {
    //   if (!bucket.empty())
    //   {
    //     reflectors.push_back(bucket);
    //     bucket.clear();
    //   }
    // }
    if ((intensities[i + 1] < intensity_lower_ || intensities[i + 1] > intensity_upper_) && is_find)
    {
      if (bucket.size() >= 2)
        reflectors.push_back(bucket);
      bucket.clear();
      is_find = false;
    }    
  }
  std::vector<int> index;
  ROS_DEBUG("reflector's size: %d", reflectors.size());
  for (int i = 0; i < reflectors.size(); ++i)
  {
    std::sort(reflectors[i].begin(), reflectors[i].end(),
              [](const std::pair<int, double>& left, const std::pair<int, double>& right)
              {
                return left.second > right.second;
              });
    std::vector<std::pair<int, double>>  item = reflectors[i];
    if (item.empty())
      continue;
    index.push_back(item[0].first); //这个地方
    ROS_DEBUG("index:%d intensity:%f size:%d",item[0].first, item[0].second, item.size());
  }
  return index;
}

std::vector<geometry_msgs::PoseStamped> LandmarksPublish::computeReflectorPose(std::vector<double> ranges,
                                                                               std::vector<int> reflectors)
{
  std::vector<geometry_msgs::PoseStamped> landmarks_pose;
  for (int i = 0; i < reflectors.size(); ++i)
  {
    double range = ranges[reflectors[i]];
    if (std::isnan(range) || std::isinf(range))
    {
      ROS_ERROR("range is nan or inf...");
      continue;
    }
    double angle = laser_angle_min_ + reflectors[i] * laser_angle_increment_;
    double x = range * cos(angle);
    double y = range * sin(angle);
    //转化tracking frame 下
    geometry_msgs::PoseStamped laser_pose;
    laser_pose.header.stamp = ros::Time(0);
    laser_pose.header.frame_id = laser_frame_;
    laser_pose.pose.position.x = x;
    laser_pose.pose.position.y = y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle);
    laser_pose.pose.orientation = tf2::toMsg(quat);
    geometry_msgs::PoseStamped tracking_frame_pose;
    if (transformPose(tracking_frame_, laser_pose, tracking_frame_pose ))
    {
      landmarks_pose.push_back(tracking_frame_pose);
    }
    else
    {
      ROS_ERROR("[computeReflectorPose] Transform failed");
    }
  }
  return landmarks_pose;
}

std::vector<std::string> LandmarksPublish::computeReflectorId(std::vector<geometry_msgs::PoseStamped> pose)
{
  std::vector<std::string> reflector_id;
  if (detect_id_mode_ == 0)
  {
      for (int i = 0; i < pose.size(); ++i)
      {
          reflector_id.push_back(std::to_string(++reflector_id_));
          if (reflector_id_ > 65535)
            reflector_id_ = 0;
      }
      return reflector_id;
  }
  else 
  {
    //pose转化到map下进行判断
    //维护一个std::map<id, int>统计某个id连续出现的次数
    for (int i = 0; i < pose.size(); ++i)
    {
      geometry_msgs::PoseStamped map_pose;
      pose[i].header.stamp = ros::Time(0);
      if (transformPose("map", pose[i], map_pose))
      {
        if (reflector_map_.empty()) //空 构造反光柱的地图
        {
          ROS_WARN("reflector map is empty, insert one reflector.");
          int id = reflector_id_++;
          reflector_map_.insert( 
           std::make_pair(std::to_string(id), map_pose));
          reflector_id.push_back(std::to_string(id));
          reflectors_mark_map_.points.push_back(map_pose.pose.position);
          // reflectors_text_mark_map_.pose = map_pose.pose;
          // reflectors_text_mark_map_.text = std::to_string(id);
          // reflectors_text_array_mark_map_.markers.push_back(reflectors_text_mark_map_);
          reflector_id_count_.insert(std::make_pair(std::to_string(id), 1));
          continue;
        }
        bool find_in_old = false;
        for (auto itr = reflector_map_.begin(); itr != reflector_map_.end(); ++itr)
        {
          double curr_x = map_pose.pose.position.x; //检测到反光柱的位置
          double curr_y = map_pose.pose.position.y;
          double ref_x = itr->second.pose.position.x; //反光柱在地图中的位置
          double ref_y = itr->second.pose.position.y;
          ROS_DEBUG("id:%s diff:(%f %f) hypot:%f", itr->first.c_str(), fabs(ref_x - curr_x), fabs(ref_y - curr_y), std::hypot(ref_x - curr_x, ref_y - curr_y));
          if (std::hypot(ref_x - curr_x, ref_y - curr_y) < close_tolerance_)
          {
            ROS_WARN("Detect Id:%s", itr->first.c_str());
            // if ((++reflector_id_count_[itr->first].second) > detect_frame_count_)
            // {
            //    reflector_id.push_back(itr->first);
            //    find_in_old = true;
            // }
            reflector_id.push_back(itr->first);
            find_in_old = true;
            //@todo找到之后要更新？
            itr->second.pose.position.x = curr_x;
            itr->second.pose.position.y = curr_y;
            itr->second.pose.orientation = map_pose.pose.orientation;
            break;
          }
        } 
        //没有找到，当做一个新的id插入到地图中
        if (!find_in_old)
        {
          int id = reflector_id_++;
          ROS_WARN("can't find this reflector, insert %d new id.", id);
          reflector_map_.insert(
          std::make_pair(std::to_string(id), map_pose));
          reflector_id.push_back(std::to_string(id));
          reflector_id_count_.insert(std::make_pair(std::to_string(id), 1));
          reflectors_mark_map_.points.push_back(map_pose.pose.position);
        }
      } 
      else
      {
        ROS_ERROR("[computeReflectorId] Transform pose failure!");
      }
    }
    //全部计算完后再进行一次判断
    std::vector<std::string> ids;
    // for (int i = 0; i < reflector_id.size(); ++i)
    // {
    //   bool is_find = false;
    //   for(auto reflector : reflector_id_count_)
    //   {
    //     auto itr = reflector_id_count_.find(reflector_id[i])
    //     if ( itr != reflector_id_count_.end()) //找到了
    //     {
    //       if ((++itr->second) > detect_frame_count_)
    //       {
    //         ids.push_back(itr->first);
    //         itr->second = 0; //大于detect_frame_count_后置为0
    //       }
    //       is_find = true;
    //       break;
    //     }
    //   }
    //   if (!is_find) //没有找到
    //    need_update.push_back(itr->first);
    // }
    for (auto itr = reflector_id_count_.begin(); itr != reflector_id_count_.end(); ++itr)
    {
      bool is_find = false;
      for (int i = 0; i < reflector_id.size(); ++i)
      {
        if (reflector_id[i] == itr->first) //当前出现了
        {
          ++itr->second;
          ROS_DEBUG("reflector id:%s detect %d times", itr->first.c_str(), itr->second);
          if (itr->second > detect_frame_count_)
          {
            ROS_ERROR("Final detect %s", itr->first.c_str());
            ids.push_back(itr->first);
            if (itr->second > 65536)
              itr->second = detect_frame_count_ + 1;
            //id.second = 0;
          }
          is_find = true;
          break;
        }
      }
      //没有出现
      if (!is_find)
      {
        itr->second = 0; //将上一时刻检测的到置为0
      }
    }
    //根据距离再筛选一次
    //std::remove_if();
    return ids;
  }
  //return reflector_id;
  
}

void LandmarksPublish::generateLandmarksData(cartographer_ros_msgs::LandmarkList& landmark_list)
{
  landmark_list.header.stamp = ros::Time::now();
  landmark_list.header.frame_id = tracking_frame_;
  if (is_pub_empty_)
   return;
  if (reflectors_pose_.empty() || reflectors_id_.empty())
  {
    ROS_ERROR("This maybe impossible......");
    return;
  }

  if (reflectors_pose_.size() != reflectors_id_.size())
  {
    ROS_ERROR("reflector'pose size mismatch reflector's id size.");
    return;
  }
  reflectors_mark_.points.clear();
  //reflectors_text_array_mark_.markers.clear();
  for (int  i = 0; i < reflectors_pose_.size(); ++i)
  {
    cartographer_ros_msgs::LandmarkEntry landmark;
    landmark.id = reflectors_id_[i];
    landmark.tracking_from_landmark_transform = reflectors_pose_[i].pose;
    landmark.translation_weight = translation_weight_;
    landmark.rotation_weight = rotation_weight_;
    landmark_list.landmarks.push_back(landmark);
    //fill landmark
    //reflectors_mark_.pose = reflectors_pose_[i].pose;
    reflectors_mark_.points.push_back(reflectors_pose_[i].pose.position);
    
    //reflectors_text_array_mark_.markers.push_back(reflectors_text_mark_);
  }
}

bool LandmarksPublish::transformPose(std::string target_frame,
                                     geometry_msgs::PoseStamped& input,
                                     geometry_msgs::PoseStamped& output)
{
  try
  {
    listener_->transformPose(target_frame, input, output);
    return true;
  }
  catch(const std::exception& e)
  {
     std::cerr << e.what() << '\n';
     return false;
  }
  
  
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "publish_landmark");
   ros::NodeHandle nh;
   LandmarksPublish landmark(nh);
   ros::spin();
   return 0;
}