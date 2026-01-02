#include "../../include/poseCaculate/poseCaculate.hpp"

tf2::Quaternion PoseCaculate::rvecToQuaternion(const cv::Mat &rvec)
{
    // 【关键修复】直接使用solvePnP返回的旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // 【重要】检查旋转矩阵是否是正交矩阵
    cv::Mat test = rotation_matrix * rotation_matrix.t();
    cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
    double diff = cv::norm(test, eye);
    
    if (diff > 0.01) {
        ROS_WARN("[Rotation] Rotation matrix not orthogonal (diff=%.4f)", diff);
    }
    
    // 转换为tf2矩阵
    tf2::Matrix3x3 tf_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
    );
    
    tf2::Quaternion quaternion;
    tf_matrix.getRotation(quaternion);
    quaternion.normalize();
    
    return quaternion;
}

void PoseCaculate::publishTfTransform(const cv::Mat &rvec, const cv::Mat &tvec,
                                     const ros::Time &stamp, int armor_id, int armor_type)
{
    if (rvec.empty() || tvec.empty()) {
        ROS_WARN("[TF] Armor %d: Empty rvec/tvec, skipping TF", armor_id);
        return;
    }
    debugTransform(rvec, tvec, armor_id);
    try {
        ros::Time transform_stamp = stamp;
        if (transform_stamp.toSec() == 0) {
            transform_stamp = ros::Time::now();
            ROS_WARN("[TF] Armor %d: Using current time (stamp was 0)", armor_id);
        }
        
        // 【关键修复】简化坐标变换逻辑
        // solvePnP返回的tvec已经是装甲板原点在相机坐标系中的坐标
        // 对于TF变换，平移向量应该是相机坐标系到装甲板坐标系的变换
        // 也就是 -R^T * tvec，但是这里有更简单的理解方式：
        
        // 1. 获取旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        // 2. 【修复】直接使用solvePnP的tvec，不进行额外的变换
        // tvec是装甲板原点在相机坐标系中的位置
        // 对于从相机到装甲板的TF变换，平移应该是-tvec（取反）
        // 但是这里需要验证方向，我们先尝试最简单的：直接使用-tvec
        
        // 创建TransformStamped消息
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = transform_stamp;
        transform_stamped.header.frame_id = parent_frame_id_;
        transform_stamped.child_frame_id = child_frame_prefix_ + std::to_string(armor_id);

        transform_stamped.transform.translation.x = tvec.at<double>(0, 0);
        transform_stamped.transform.translation.y = tvec.at<double>(1, 0);
        transform_stamped.transform.translation.z = tvec.at<double>(2, 0);
        
        // 3. 计算旋转（保持之前的修正）
        // solvePnP返回的是从装甲板坐标系到相机坐标系的旋转
        // 对于TF，我们需要从相机坐标系到装甲板坐标系的旋转
        cv::Mat rotation_matrix_inv = rotation_matrix.t();  // 取转置得到逆旋转
        
        tf2::Matrix3x3 tf_matrix(
            rotation_matrix_inv.at<double>(0, 0), rotation_matrix_inv.at<double>(0, 1), rotation_matrix_inv.at<double>(0, 2),
            rotation_matrix_inv.at<double>(1, 0), rotation_matrix_inv.at<double>(1, 1), rotation_matrix_inv.at<double>(1, 2),
            rotation_matrix_inv.at<double>(2, 0), rotation_matrix_inv.at<double>(2, 1), rotation_matrix_inv.at<double>(2, 2)
        );
        
        tf2::Quaternion quaternion;
        tf_matrix.getRotation(quaternion);
        quaternion.normalize();
        
        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();
        
        // 记录活动ID
        active_armor_ids_.insert(armor_id);
        last_tf_time_ = ros::Time::now();
        
        // 发布TF变换
        tf_broadcaster_->sendTransform(transform_stamped);
        
        if (debug_mode_) {
            ROS_INFO("[TF] Published: %s -> %s", 
                     parent_frame_id_.c_str(), transform_stamped.child_frame_id.c_str());
            ROS_INFO("[TF] Translation: [%.3f, %.3f, %.3f]", 
                     transform_stamped.transform.translation.x,
                     transform_stamped.transform.translation.y,
                     transform_stamped.transform.translation.z);
        }
    }
    catch (const cv::Exception &e) {
        ROS_ERROR("[TF] CV exception for armor %d: %s", armor_id, e.what());
    }
    catch (const std::exception &e) {
        ROS_ERROR("[TF] Exception for armor %d: %s", armor_id, e.what());
    }
}

void PoseCaculate::publishPoseMessage(const cv::Mat &rvec, const cv::Mat &tvec,
                                     const ros::Time &stamp, int armor_id, int armor_type)
{
    try {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = parent_frame_id_;
        
        pose_msg.pose.position.x = tvec.at<double>(0, 0);
        pose_msg.pose.position.y = tvec.at<double>(1, 0);
        pose_msg.pose.position.z = tvec.at<double>(2, 0);
        
        tf2::Quaternion quaternion = rvecToQuaternion(rvec);
        pose_msg.pose.orientation.x = quaternion.x();
        pose_msg.pose.orientation.y = quaternion.y();
        pose_msg.pose.orientation.z = quaternion.z();
        pose_msg.pose.orientation.w = quaternion.w();
        
        pose_msg.header.seq = armor_id;
        pose_pub_.publish(pose_msg);
        
        if (debug_mode_) {
            ROS_DEBUG("[Pose] Published for armor %d", armor_id);
        }
    }
    catch (const std::exception &e) {
        ROS_ERROR("[Pose] Exception for armor %d: %s", armor_id, e.what());
    }
}

void PoseCaculate::printRotationInfo(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id)
{
    if (!debug_mode_) return;
    
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // 【关键】打印旋转矩阵各列（装甲板坐标系各轴在相机坐标系中的方向）
    cv::Mat x_axis = rotation_matrix.col(0);  // 装甲板X轴
    cv::Mat y_axis = rotation_matrix.col(1);  // 装甲板Y轴
    cv::Mat z_axis = rotation_matrix.col(2);  // 装甲板Z轴
    
    ROS_INFO("[Rotation] Armor %d axes in camera frame:", armor_id);
    ROS_INFO("  X-axis (right): [%.3f, %.3f, %.3f]", 
             x_axis.at<double>(0), x_axis.at<double>(1), x_axis.at<double>(2));
    ROS_INFO("  Y-axis (down):  [%.3f, %.3f, %.3f]", 
             y_axis.at<double>(0), y_axis.at<double>(1), y_axis.at<double>(2));
    ROS_INFO("  Z-axis (out):   [%.3f, %.3f, %.3f]", 
             z_axis.at<double>(0), z_axis.at<double>(1), z_axis.at<double>(2));
    
    // 计算装甲板Z轴与相机Z轴（[0,0,1]）的夹角
    cv::Mat camera_z = (cv::Mat_<double>(3,1) << 0, 0, 1);
    double dot_product = z_axis.dot(camera_z);
    double norm_z = cv::norm(z_axis);
    double angle = acos(dot_product / norm_z) * 180.0 / CV_PI;
    
    ROS_INFO("[Rotation] Angle between armor Z and camera Z: %.1f°", angle);
    
    // 检查Z轴是否大致指向相机（与相机Z轴夹角应较小）
    if (angle > 90.0) {
        ROS_WARN("[Rotation] Armor %d: Z axis pointing away from camera (angle=%.1f°)", 
                 armor_id, angle);
    }
}

void PoseCaculate::validateCoordinateSystem(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id)
{
    if (!debug_mode_) return;
    
    // 将3D模型点变换到相机坐标系
    std::vector<cv::Point3f> object_points = get3DObjectPoints(0); // 假设是小装甲板
    
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    ROS_INFO("[Validate] Armor %d coordinate check:", armor_id);
    
    // 检查每个点的Z坐标（应该是正数，表示在相机前方）
    for (int i = 0; i < object_points.size(); ++i) {
        cv::Mat point_mat = (cv::Mat_<double>(3,1) << 
                            object_points[i].x, 
                            object_points[i].y, 
                            object_points[i].z);
        
        cv::Mat transformed = rotation_matrix * point_mat + tvec;
        
        ROS_INFO("  Point %d in camera: [%.3f, %.3f, %.3f]", 
                 i, 
                 transformed.at<double>(0),
                 transformed.at<double>(1),
                 transformed.at<double>(2));
    }
}

void PoseCaculate::debugTransform(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id)
{
    if (!debug_mode_) return;
    
    ROS_INFO("[Debug] Armor %d transform analysis:", armor_id);
    ROS_INFO("  solvePnP tvec (armor in camera): [%.3f, %.3f, %.3f]", 
             tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // 测试不同的变换方式
    // 方式1：直接使用tvec
    ROS_INFO("  Method 1 (direct): translation = [%.3f, %.3f, %.3f]", 
             tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    
    // 方式2：使用 -tvec
    ROS_INFO("  Method 2 (negated): translation = [%.3f, %.3f, %.3f]", 
             -tvec.at<double>(0), -tvec.at<double>(1), -tvec.at<double>(2));
    
    // 方式3：使用 -R^T * tvec（之前的错误方式）
    cv::Mat rotation_matrix_trans = rotation_matrix.t();
    cv::Mat tvec_old = -rotation_matrix_trans * tvec;
    ROS_INFO("  Method 3 (-R^T*tvec): translation = [%.3f, %.3f, %.3f]", 
             tvec_old.at<double>(0), tvec_old.at<double>(1), tvec_old.at<double>(2));
}