#include "../../include/poseCaculate/poseCaculate.hpp"
bool PoseCaculate::solvePnPForArmor(const armor_detect::ArmorInfo &armor,cv::Mat &rvec, cv::Mat &tvec)
{
    // 1. 提取2D像素点
    std::vector<cv::Point2f> image_points;
    if (!extractImagePoints(armor, image_points))
    {
        ROS_WARN("Failed to extract image points for armor %d", armor.armor_id);
        return false;
    }
    
    // 2. 获取对应的3D模型点
    std::vector<cv::Point3f> object_points = get3DObjectPoints(armor.armor_type);
    
    // 3. 调用solvePnP
    try
    {
        bool success = cv::solvePnP(object_points, image_points,
                                    camera_matrix_, dist_coeffs_,
                                    rvec, tvec, false, pnp_method_);
        
        if (!success)
        {
            if (debug_mode_)
            {
                ROS_WARN("solvePnP failed for armor %d", armor.armor_id);
            }
            return false;
        }
        
        // 4. 验证结果合理性
        if (!validatePoseResult(rvec, tvec, armor.armor_id))
        {
            return false;
        }
        
        // 5. 计算重投影误差（可选，用于调试）
        if (debug_mode_)
        {
            double reproj_error = calculateReprojectionError(
                object_points, image_points, rvec, tvec);
            ROS_DEBUG("Armor %d reprojection error: %.2f pixels", 
                        armor.armor_id, reproj_error);
        }
        
        return true;
    }
    catch (const cv::Exception &e)
    {
        ROS_ERROR("OpenCV exception in solvePnP: %s", e.what());
        return false;
    }
}


bool PoseCaculate::extractImagePoints(const armor_detect::ArmorInfo &armor,std::vector<cv::Point2f> &image_points)
{
    // 清空输出向量
    image_points.clear();
    
    // 从消息中提取4个顶点
    // 点顺序是：左上->右上->右下->左下
    for (int i = 0; i < 4; ++i)
    {
        cv::Point2f point(armor.vertices_pixel[i].x,
                            armor.vertices_pixel[i].y);
        
        // 验证点是否有效（非负坐标）
        if (point.x < 0 || point.y < 0)
        {
            ROS_WARN_THROTTLE(1.0, "Invalid point coordinates for armor %d", 
                                armor.armor_id);
            return false;
        }
        
        image_points.push_back(point);
    }
    
    // 验证是否有4个点
    if (image_points.size() != 4)
    {
        ROS_ERROR("Expected 4 points, got %zu", image_points.size());
        return false;
    }
    
    return true;
}


bool PoseCaculate::validatePoseResult(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id)
{
    // 检查矩阵是否为空
    if (rvec.empty() || tvec.empty())
    {
        ROS_WARN("Empty rotation or translation vector for armor %d", armor_id);
        return false;
    }
    
    // 获取距离（Z轴分量）
    double distance = tvec.at<double>(2, 0);
    
    // 检查距离是否合理
    if (distance < min_valid_distance_ || distance > max_valid_distance_)
    {
        if (debug_mode_)
        {
            ROS_WARN("Armor %d distance out of range: %.2f m (min=%.1f, max=%.1f)", 
                        armor_id, distance, min_valid_distance_, max_valid_distance_);
        }
        return false;
    }
    
    // 检查平移向量分量是否在合理范围内
    double x = tvec.at<double>(0, 0);
    double y = tvec.at<double>(1, 0);
    
    // 假设相机视野在X、Y方向上的最大范围是距离的2倍（tan(45°)=1）
    double max_xy = distance * 1.5;  // 稍微宽松一点
    
    if (fabs(x) > max_xy || fabs(y) > max_xy)
    {
        if (debug_mode_)
        {
            ROS_WARN("Armor %d position out of FOV: x=%.2f, y=%.2f, z=%.2f", 
                        armor_id, x, y, distance);
        }
        return false;
    }
    
    return true;
}


double PoseCaculate::calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                    const std::vector<cv::Point2f> &image_points,
                                    const cv::Mat &rvec, const cv::Mat &tvec)
{
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, 
                        camera_matrix_, dist_coeffs_, 
                        projected_points);
    
    double total_error = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i)
    {
        double error = cv::norm(image_points[i] - projected_points[i]);
        total_error += error;
    }
    
    return total_error / image_points.size();
}

void PoseCaculate::printPoseResult(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id, int armor_type)
{
    // 计算欧几里得距离
    double distance = cv::norm(tvec);
    
    // 获取具体坐标
    double x = tvec.at<double>(0, 0);
    double y = tvec.at<double>(1, 0);
    double z = tvec.at<double>(2, 0);
    
    // 输出结果
    ROS_INFO("=========================================");
    ROS_INFO("ARMOR %d (%s)", armor_id, 
                armor_type == 0 ? "SMALL" : "BIG");
    ROS_INFO("  Position in camera frame:");
    ROS_INFO("    X: %7.3f m  (right/left)", x);
    ROS_INFO("    Y: %7.3f m  (down/up)", y);
    ROS_INFO("    Z: %7.3f m  (forward)", z);
    ROS_INFO("  Distance: %7.3f m", distance);
    ROS_INFO("  Rotation vector:");
    ROS_INFO("    [%8.5f, %8.5f, %8.5f]", 
                rvec.at<double>(0, 0), 
                rvec.at<double>(1, 0), 
                rvec.at<double>(2, 0));
    ROS_INFO("=========================================");
}    