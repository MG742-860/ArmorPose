#include "../../include/poseCaculate/poseCaculate.hpp"

PoseCaculate::PoseCaculate(ros::NodeHandle &nh) : nh_(nh)
{
    ROS_INFO("Initializing PoseCaculate...");
    
    // 1. 从参数服务器加载参数
    loadParameters();
    // 2. 订阅相机信息（单次）
    camera_info_sub_ = nh_.subscribe("/hk_camera/camera_info", 1, &PoseCaculate::cameraInfoCallback, this);
    // 3. 订阅装甲板检测结果
    armor_sub_ = nh_.subscribe("/ArmorDetect/armors", 10, &PoseCaculate::armorCallback, this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/armor_pose", 10);
    tf_cleanup_timer_ = nh_.createTimer(ros::Duration(0.01),&PoseCaculate::cleanupOldTf, this);
    // 4. 生成3D模型点
    generate3DPoints();// 装甲板不会变，可以复用，只需生成一次
    ROS_INFO("PoseCaculate initialized successfully");
    ROS_INFO("Waiting for camera info...");
}

void PoseCaculate::loadParameters()
{
    // 装甲板尺寸
    small_armor_width_ = nh_.param("small_armor_width", 0.135);
    small_armor_height_ = nh_.param("small_armor_height", 0.060);
    big_armor_width_ = nh_.param("big_armor_width", 0.230);
    big_armor_height_ = nh_.param("big_armor_height", 0.060);
    
    // PnP配置
    pnp_method_ = nh_.param("pnp_method", 1);
    
    // 调试选项
    debug_mode_ = nh_.param("debug_mode", true);
    print_results_ = nh_.param("print_results", true);
    min_valid_distance_ = nh_.param("min_valid_distance", 0.2);
    max_valid_distance_ = nh_.param("max_valid_distance", 10.0);
    
    // TF配置
    nh_.param("publish_tf", publish_tf_, true);
    nh_.param("publish_pose_messages", publish_pose_messages_, true);
    nh_.param("parent_frame_id", parent_frame_id_, std::string("camera_optical_frame"));
    nh_.param("child_frame_prefix", child_frame_prefix_, std::string("armor_"));
    nh_.param("tf_cache_time", tf_cache_time_, 0.1);
    
    if (debug_mode_) {
        ROS_INFO("[Params] Armor: Small=%.3fx%.3fm, Big=%.3fx%.3fm",
                small_armor_width_, small_armor_height_,
                big_armor_width_, big_armor_height_);
    }
}

void PoseCaculate::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if (camera_info_set_) {
        ROS_WARN_THROTTLE(5.0, "[Camera] Info already set, ignoring");
        return;
    }
    
    // 内参矩阵
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->K[0];  // fx
    camera_matrix_.at<double>(0, 2) = msg->K[2];  // cx
    camera_matrix_.at<double>(1, 1) = msg->K[4];  // fy
    camera_matrix_.at<double>(1, 2) = msg->K[5];  // cy
    
    // 畸变系数
    if (!msg->D.empty()) {
        dist_coeffs_ = cv::Mat(msg->D).clone();
    } else {
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        ROS_WARN("[Camera] No distortion coefficients, using zeros");
    }
    
    // 验证内参
    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    
    if (fx <= 0 || fy <= 0) {
        ROS_ERROR("[Camera] FATAL: Invalid focal length (fx=%.1f, fy=%.1f)", fx, fy);
        ROS_ERROR("[Camera] PnP will fail! Check calibration!");
        return;
    }
    
    camera_info_set_ = true;
    
    ROS_INFO("[Camera] Parameters set: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
            fx, fy,
            camera_matrix_.at<double>(0, 2),
            camera_matrix_.at<double>(1, 2));
}

void PoseCaculate::generate3DPoints()
{    
    // 小装甲板3D点
    double sw2 = small_armor_width_ / 2;
    double sh2 = small_armor_height_ / 2;
    
    small_armor_points_.clear();
    // 注意：这里Y坐标取负，因为图像坐标Y轴向下，但世界坐标Y轴向上
    small_armor_points_.push_back(cv::Point3f(-sw2, -sh2, 0));  // 左上
    small_armor_points_.push_back(cv::Point3f(sw2, -sh2, 0));   // 右上
    small_armor_points_.push_back(cv::Point3f(sw2, sh2, 0));    // 右下
    small_armor_points_.push_back(cv::Point3f(-sw2, sh2, 0));   // 左下
    
    // 大装甲板3D点
    double bw2 = big_armor_width_ / 2;
    double bh2 = big_armor_height_ / 2;
    
    big_armor_points_.clear();
    big_armor_points_.push_back(cv::Point3f(-bw2, -bh2, 0));    // 左上
    big_armor_points_.push_back(cv::Point3f(bw2, -bh2, 0));     // 右上
    big_armor_points_.push_back(cv::Point3f(bw2, bh2, 0));      // 右下
    big_armor_points_.push_back(cv::Point3f(-bw2, bh2, 0));     // 左下
    
    if (debug_mode_)
    {
        ROS_INFO("[3D] Model points generated:");
        for (int i = 0; i < 4; ++i) {
            ROS_INFO("  Point %d: (%.3f, %.3f, %.3f)", 
                     i, small_armor_points_[i].x, 
                     small_armor_points_[i].y, small_armor_points_[i].z);
        }
    }
}

std::vector<cv::Point3f> PoseCaculate::get3DObjectPoints(int armor_type)
{
    if (armor_type == 0)  // 小装甲板
    {
        return small_armor_points_;
    }
    else if (armor_type == 1)  // 大装甲板
    {
        return big_armor_points_;
    }
    else
    {
        ROS_ERROR("Invalid armor type: %d, using small armor", armor_type);
        return small_armor_points_;
    }
}

void PoseCaculate::armorCallback(const armor_detect::ArmorArrayConstPtr &armor_msg)
{
    if (!camera_info_set_) {
        ROS_WARN_THROTTLE(2.0, "[PnP] Camera info not ready, skipping frame");
        return;
    }
    
    if (!armor_msg || armor_msg->armors.empty()) {
        if (debug_mode_) {
            ROS_DEBUG("[PnP] Empty armor message");
        }
        return;
    }
    ROS_INFO("=== Raw armor data ===");
    ROS_INFO("Armor ID: %d", armor_msg->armors[0].armor_id);
    ROS_INFO("Vertices (raw order):");
    for (int i = 0; i < 4; i++) {
        ROS_INFO("  [%d]: (%.1f, %.1f)", i, 
                 armor_msg->armors[0].vertices_pixel[i].x, armor_msg->armors[0].vertices_pixel[i].y);
    }
    // 处理每个装甲板
    int success_count = 0;
    for (size_t i = 0; i < armor_msg->armors.size(); ++i) {
        const auto &armor = armor_msg->armors[i];
        
        cv::Mat rvec, tvec;
        if (solvePnPForArmor(armor, rvec, tvec)) {
            success_count++;
            
            if (print_results_) {
                printPoseResult(rvec, tvec, armor.armor_id, armor.armor_type);
            }
            
            if (publish_tf_ && tf_broadcaster_) {
                publishTfTransform(rvec, tvec, armor_msg->header.stamp, 
                                  armor.armor_id, armor.armor_type);
            }
            
            if (publish_pose_messages_) {
                publishPoseMessage(rvec, tvec, armor_msg->header.stamp,
                                  armor.armor_id, armor.armor_type);
            }
        }
    }
    
    if (debug_mode_ && success_count > 0) {
        ROS_DEBUG("[PnP] Frame processed: %d/%zu armors solved",
                 success_count, armor_msg->armors.size());
    }
}

void PoseCaculate::cleanupOldTf(const ros::TimerEvent& event)
{
    if (!publish_tf_ || !tf_broadcaster_) return;
    
    ros::Time now = ros::Time::now();
    
    // 如果超过2秒没更新，从活动列表中移除
    if ((now - last_tf_time_).toSec() > 2.0) {
        active_armor_ids_.clear();
        if (debug_mode_) {
            ROS_DEBUG("[TF] Cleared inactive armor IDs");
        }
    }
}