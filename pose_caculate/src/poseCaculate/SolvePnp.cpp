#include "../../include/poseCaculate/poseCaculate.hpp"

bool PoseCaculate::solvePnPForArmor(const armor_detect::ArmorInfo &armor, cv::Mat &rvec, cv::Mat &tvec)
{
    // 1. 提取2D像素点
    std::vector<cv::Point2f> image_points;
    if (!extractImagePoints(armor, image_points)) {
        if (debug_mode_) {
            ROS_WARN("[PnP] Armor %d: Failed to extract 2D points", armor.armor_id);
        }
        return false;
    }
    
    // 2. 获取3D模型点
    std::vector<cv::Point3f> object_points = get3DObjectPoints(armor.armor_type);
    
    // 3. 检查2D点面积
    cv::RotatedRect min_rect = cv::minAreaRect(image_points);
    float rect_area = min_rect.size.width * min_rect.size.height;
    if (rect_area < 100.0) {
        if (debug_mode_) {
            ROS_WARN("[PnP] Armor %d: 2D area too small (%.1f)", armor.armor_id, rect_area);
        }
        return false;
    }
    
    // 4. 调用solvePnP
    try 
    {
        bool success = cv::solvePnP(object_points, image_points,
                                    camera_matrix_, dist_coeffs_,
                                    rvec, tvec, false, pnp_method_);
        
        if (!success) 
        {
            if (debug_mode_) 
            {
                ROS_WARN("[PnP] Armor %d: solvePnP failed", armor.armor_id);
            }
            return false;
        }
        // 5. 检查距离合理性
        double current_distance = cv::norm(tvec);
        if (current_distance < min_valid_distance_ || current_distance > max_valid_distance_) 
        {
            return false;
        }
        
        // 6. 验证位姿
        if (!validatePoseResult(rvec, tvec, armor.armor_id)) 
        {
            return false;
        }
        static std::map<int, cv::Mat> last_tvecs;
        static std::map<int, cv::Mat> last_rvecs;
        int id = armor.armor_id;
        
        // 更新历史记录
        last_tvecs[id] = tvec.clone();
        last_rvecs[id] = rvec.clone();

        return true;
    }catch (const cv::Exception &e) 
    {
        ROS_ERROR("[PnP] OpenCV exception: %s", e.what());
        return false;
    }
}

bool PoseCaculate::extractImagePoints(const armor_detect::ArmorInfo &armor,
                                      std::vector<cv::Point2f> &image_points)
{
    image_points.clear();
    if(armor.vertices_pixel.size() != 4) return false;
    for (size_t i = 0; i < 4; i++)
    {
        image_points.push_back(cv::Point2f(armor.vertices_pixel[i].x, armor.vertices_pixel[i].y));
    }
    return true;
}

bool PoseCaculate::validatePoseResult(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id)
{
    if (rvec.empty() || tvec.empty()) {
        ROS_WARN("[Validate] Armor %d: Empty rvec/tvec", armor_id);
        return false;
    }
    
    double distance = tvec.at<double>(2, 0);
    
    if (distance < min_valid_distance_ || distance > max_valid_distance_) {
        if (debug_mode_) {
            ROS_WARN("[Validate] Armor %d: Distance %.2fm out of range", 
                    armor_id, distance);
        }
        return false;
    }
    
    return true;
}

void PoseCaculate::printPoseResult(const cv::Mat &rvec, const cv::Mat &tvec, 
                                   int armor_id, int armor_type)
{
    double distance = cv::norm(tvec);
    double x = tvec.at<double>(0, 0);
    double y = tvec.at<double>(1, 0);
    double z = tvec.at<double>(2, 0);
    
    ROS_INFO("[Result] Armor %d (%s): Pos=[%.3f, %.3f, %.3f]m, Dist=%.3fm",
            armor_id,
            armor_type == 0 ? "SMALL" : "BIG",
            x, y, z, distance);
}    