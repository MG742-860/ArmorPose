#include "../../include/poseCaculate/poseCaculate.hpp"

tf2::Quaternion PoseCaculate::rvecToQuaternion(const cv::Mat &rvec)
{
    // 将旋转向量转换为旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // 注意：solvePnP返回的旋转矩阵是从装甲板坐标系到相机坐标系的变换
    // 对于TF，我们需要的是从相机坐标系到装甲板坐标系的变换（逆变换）
    // 旋转矩阵的逆 = 转置（因为旋转矩阵是正交矩阵）
    cv::Mat rotation_matrix_inv = rotation_matrix.t();
    
    // 将OpenCV矩阵转换为tf2::Matrix3x3
    tf2::Matrix3x3 tf_matrix(
        rotation_matrix_inv.at<double>(0, 0), rotation_matrix_inv.at<double>(0, 1), rotation_matrix_inv.at<double>(0, 2),
        rotation_matrix_inv.at<double>(1, 0), rotation_matrix_inv.at<double>(1, 1), rotation_matrix_inv.at<double>(1, 2),
        rotation_matrix_inv.at<double>(2, 0), rotation_matrix_inv.at<double>(2, 1), rotation_matrix_inv.at<double>(2, 2)
    );
    
    // 转换为四元数
    tf2::Quaternion quaternion;
    tf_matrix.getRotation(quaternion);
    
    // 确保四元数是单位四元数
    quaternion.normalize();
    
    return quaternion;
}

void PoseCaculate::publishTfTransform(const cv::Mat &rvec, const cv::Mat &tvec,
                                     const ros::Time &stamp, int armor_id, int armor_type)
{
    try
    {
        // 1. 将旋转向量转换为四元数
        tf2::Quaternion quaternion = rvecToQuaternion(rvec);
        
        // 2. 计算平移向量的逆变换
        //    tvec是装甲板原点在相机坐标系下的坐标
        //    对于从相机到装甲板的变换，平移向量应该是 -R_inv * tvec
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        cv::Mat rotation_matrix_inv = rotation_matrix.t();
        cv::Mat tvec_inv = -rotation_matrix_inv * tvec;
        
        // 3. 创建TransformStamped消息
        geometry_msgs::TransformStamped transform_stamped;
        
        // 设置时间戳
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = parent_frame_id_;
        
        // 设置子坐标系名称（例如：armor_0, armor_1等）
        std::string child_frame_id = child_frame_prefix_ + std::to_string(armor_id);
        transform_stamped.child_frame_id = child_frame_id;
        
        // 设置平移
        transform_stamped.transform.translation.x = tvec_inv.at<double>(0, 0);
        transform_stamped.transform.translation.y = tvec_inv.at<double>(1, 0);
        transform_stamped.transform.translation.z = tvec_inv.at<double>(2, 0);
        
        // 设置旋转（四元数）
        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();
        
        // 4. 发布TF变换
        tf_broadcaster_->sendTransform(transform_stamped);
        
        if (debug_mode_)
        {
            ROS_INFO("Published TF: %s -> %s", 
                     parent_frame_id_.c_str(), child_frame_id.c_str());
        }
    }
    catch (const cv::Exception &e)
    {
        ROS_ERROR("CV exception in TF publishing for armor %d: %s", 
                  armor_id, e.what());
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception in TF publishing for armor %d: %s", 
                  armor_id, e.what());
    }
}

void PoseCaculate::publishPoseMessage(const cv::Mat &rvec, const cv::Mat &tvec,
                                     const ros::Time &stamp, int armor_id, int armor_type)
{
    try
    {
        geometry_msgs::PoseStamped pose_msg;
        
        // 设置头信息
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = parent_frame_id_;
        
        // 设置位置（tvec是装甲板在相机坐标系下的坐标）
        pose_msg.pose.position.x = tvec.at<double>(0, 0);
        pose_msg.pose.position.y = tvec.at<double>(1, 0);
        pose_msg.pose.position.z = tvec.at<double>(2, 0);
        
        // 设置方向（转换为四元数）
        tf2::Quaternion quaternion = rvecToQuaternion(rvec);
        pose_msg.pose.orientation.x = quaternion.x();
        pose_msg.pose.orientation.y = quaternion.y();
        pose_msg.pose.orientation.z = quaternion.z();
        pose_msg.pose.orientation.w = quaternion.w();
        
        // 可选：添加一些额外信息到ID字段
        pose_msg.header.seq = armor_id;
        
        // 发布消息
        pose_pub_.publish(pose_msg);
        
        if (debug_mode_)
        {
            ROS_DEBUG("Published PoseStamped for armor %d", armor_id);
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception in pose message publishing for armor %d: %s", 
                  armor_id, e.what());
    }
}