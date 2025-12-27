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
    // 4. 生成3D模型点
    generate3DPoints();
    ROS_INFO("PoseCaculate initialized successfully");
    ROS_INFO("Waiting for camera info...");
    // =============

}

void PoseCaculate::loadParameters()
{
    ROS_INFO("Loading parameters from ROS parameter server...");
    
    // 装甲板物理尺寸（单位：米）
    small_armor_width_ = nh_.param("small_armor_width", 0.230);
    small_armor_height_ = nh_.param("small_armor_height", 0.127);
    big_armor_width_ = nh_.param("big_armor_width", 0.330);
    big_armor_height_ = nh_.param("big_armor_height", 0.127);

    // PnP解算方法
    pnp_method_ = nh_.param("pnp_method", 1/*cv::SOLVEPNP_EPNP*/);
    
    // 调试选项
    debug_mode_ = nh_.param("debug_mode", true);
    print_results_ = nh_.param("print_results", true);
    min_valid_distance_ = nh_.param("min_valid_distance", 0.2);
    max_valid_distance_ = nh_.param("max_valid_distance", 10.0);
    
    // TF发布配置
    nh_.param("publish_tf", publish_tf_, true);
    nh_.param("publish_pose_messages", publish_pose_messages_, true);
    nh_.param("parent_frame_id", parent_frame_id_, std::string("camera_optical_frame"));
    nh_.param("child_frame_prefix", child_frame_prefix_, std::string("armor_"));
    // TF过期时间（秒）
    nh_.param("tf_cache_time", tf_cache_time_, 0.1);


    ROS_INFO("Armor dimensions loaded:");
    ROS_INFO("  Small: %.3f x %.3f m", small_armor_width_, small_armor_height_);
    ROS_INFO("  Big: %.3f x %.3f m", big_armor_width_, big_armor_height_);
    ROS_INFO("PnP method: %d", pnp_method_);
}

void PoseCaculate::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    // 只设置一次相机参数
    if (camera_info_set_)
    {
        ROS_WARN_THROTTLE(5.0, "Camera info already set, ignoring new message");
        return;
    }
    
    // 从CameraInfo消息提取内参矩阵
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->K[0];  // fx
    camera_matrix_.at<double>(0, 1) = msg->K[1];  // skew (通常为0)
    camera_matrix_.at<double>(0, 2) = msg->K[2];  // cx
    camera_matrix_.at<double>(1, 0) = msg->K[3];  // 0
    camera_matrix_.at<double>(1, 1) = msg->K[4];  // fy
    camera_matrix_.at<double>(1, 2) = msg->K[5];  // cy
    
    // 提取畸变系数
    if (!msg->D.empty())
    {
        dist_coeffs_ = cv::Mat(msg->D).clone();
        ROS_INFO("Loaded %ld distortion coefficients", msg->D.size());
    }
    else
    {
        // 如果没有畸变系数，创建空的畸变系数矩阵
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        ROS_WARN("No distortion coefficients provided, using zeros");
    }
    
    camera_info_set_ = true;
    
    ROS_INFO("===========================================");
    ROS_INFO("CAMERA PARAMETERS SET");
    ROS_INFO("  fx: %.2f, fy: %.2f", 
                camera_matrix_.at<double>(0, 0), 
                camera_matrix_.at<double>(1, 1));
    ROS_INFO("  cx: %.2f, cy: %.2f", 
                camera_matrix_.at<double>(0, 2), 
                camera_matrix_.at<double>(1, 2));
    ROS_INFO("===========================================");
}

void PoseCaculate::generate3DPoints()
{
    // 小装甲板3D点（以装甲板中心为原点，Z=0平面）
    // 顺序：左上 -> 右上 -> 右下 -> 左下
    double sw2 = small_armor_width_ / 2;
    double sh2 = small_armor_height_ / 2;
    
    small_armor_points_.clear();
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
        ROS_INFO("3D model points generated:");
        ROS_INFO("  Small armor: 4 points");
        ROS_INFO("  Big armor: 4 points");
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
    // 检查相机参数是否已设置
    if (!camera_info_set_)
    {
        ROS_WARN_THROTTLE(1.0, "Camera info not received, skipping frame");
        return;
    }
    
    // 检查消息是否有效
    if (!armor_msg)
    {
        ROS_WARN("Received empty armor message");
        return;
    }
    
    // 统计处理
    static int frame_count = 0;
    frame_count++;
    
    if (debug_mode_)
    {
        ROS_INFO("=== Processing frame %d ===", frame_count);
        ROS_INFO("Detected %zu armors", armor_msg->armors.size());
    }
    
    // 处理每个检测到的装甲板
    int success_count = 0;
    for (size_t i = 0; i < armor_msg->armors.size(); ++i)
    {
        const auto &armor = armor_msg->armors[i];
        
        // 解算位姿
        cv::Mat rvec, tvec;
        bool success = solvePnPForArmor(armor, rvec, tvec);
        
        if (success)
        {
            success_count++;
            
            // 打印结果
            if (print_results_)
            {
                printPoseResult(rvec, tvec, armor.armor_id, armor.armor_type);
            }
            // 【关键修复】发布TF变换
            if (publish_tf_ && tf_broadcaster_) {
                publishTfTransform(rvec, tvec, armor_msg->header.stamp, 
                                  armor.armor_id, armor.armor_type);
            }
            
            // 【关键修复】发布姿态消息
            if (publish_pose_messages_) {
                publishPoseMessage(rvec, tvec, armor_msg->header.stamp, 
                                  armor.armor_id, armor.armor_type);
            }
            // 这里可以存储结果或进行后续处理
            // 例如：存储到容器中供其他函数使用
        }
    }
    
    if (debug_mode_ && armor_msg->armors.size() > 0)
    {
        ROS_INFO("Successfully solved %d/%zu armors", 
                    success_count, armor_msg->armors.size());
        ROS_INFO("Frame processed at time: %.6f\n", 
                    armor_msg->header.stamp.toSec());
    }
}