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
    
    // 3. 检查2D点质量
    cv::RotatedRect min_rect = cv::minAreaRect(image_points);
    float rect_area = min_rect.size.width * min_rect.size.height;
    
    if (rect_area < 100.0) {
        if (debug_mode_) {
            ROS_WARN("[PnP] Armor %d: 2D area too small (%.1f)", armor.armor_id, rect_area);
        }
        return false;
    }
    
    // 4. 调用solvePnP
    try {
        bool success = cv::solvePnP(object_points, image_points,
                                    camera_matrix_, dist_coeffs_,
                                    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        if (!success) {
            if (debug_mode_) {
                ROS_WARN("[PnP] Armor %d: solvePnP failed", armor.armor_id);
            }
            return false;
        }
        
        // 5. 检查距离合理性
        double distance = cv::norm(tvec);
        if (distance > 1000.0) {
            ROS_WARN("[PnP] Armor %d: Extreme distance %.1fm (rejected)", 
                     armor.armor_id, distance);
            return false;
        }
        
        // 6. 验证位姿
        if (!validatePoseResult(rvec, tvec, armor.armor_id)) {
            return false;
        }
        
        // 7. 【增强滤波】使用更强的滤波和跳变检测
        static std::map<int, cv::Mat> prev_tvecs;
        static std::map<int, cv::Mat> prev_rvecs;
        static std::map<int, int> valid_frame_count;
        static std::map<int, std::deque<double>> distance_history;  // 距离历史队列
        
        int armor_id = armor.armor_id;
        
        // 初始化
        if (valid_frame_count.find(armor_id) == valid_frame_count.end()) {
            valid_frame_count[armor_id] = 0;
            distance_history[armor_id] = std::deque<double>();
        }
        
        // 获取当前距离
        double current_distance = cv::norm(tvec);

        // 更新距离历史（保持最近10帧）
        distance_history[armor_id].push_back(current_distance);
        if (distance_history[armor_id].size() > 10) {
            distance_history[armor_id].pop_front();
        }
        
        // 【增强滤波】使用自适应的滤波系数
        double filter_alpha = 0.5;  // 基础滤波系数
        
        // 如果距离变化剧烈，使用更强的滤波
        if (!distance_history[armor_id].empty() && distance_history[armor_id].size() >= 3) {
            double last_distance = distance_history[armor_id].back();
            double second_last = distance_history[armor_id][distance_history[armor_id].size()-2];
            double distance_change = fabs(last_distance - second_last);
            
            if (distance_change > 0.1) {  // 距离变化超过0.1米
                filter_alpha = 0.7;  // 更强的滤波
                if (debug_mode_) {
                    ROS_DEBUG("[PnP] Armor %d: Using stronger filter (alpha=%.1f) due to distance change %.3f",
                            armor_id, filter_alpha, distance_change);
                }
            }
        }
        
        // 应用滤波
        if (valid_frame_count[armor_id] >= 5 && 
            prev_tvecs.find(armor_id) != prev_tvecs.end()) {
            
            // 对平移向量滤波
            cv::Mat filtered_tvec = filter_alpha * tvec + (1.0 - filter_alpha) * prev_tvecs[armor_id];
            
            // 【新增】对距离进行独立约束
            double filtered_distance = cv::norm(filtered_tvec);
            double original_distance = cv::norm(tvec);
            
            // 如果滤波后距离变化太大，限制变化幅度
            if (fabs(filtered_distance - original_distance) > 0.3) {
                if (debug_mode_) {
                    ROS_DEBUG("[PnP] Armor %d: Limiting filter effect (change: %.3f -> %.3f)",
                            armor_id, original_distance, filtered_distance);
                }
                // 使用较小的变化
                filtered_tvec = 0.8 * tvec + 0.2 * prev_tvecs[armor_id];
            }
            
            tvec = filtered_tvec;
            rvec = filter_alpha * rvec + (1.0 - filter_alpha) * prev_rvecs[armor_id];
        }
        
        // 更新计数器和历史数据
        valid_frame_count[armor_id]++;
        prev_tvecs[armor_id] = tvec.clone();
        prev_rvecs[armor_id] = rvec.clone();
        
        if (debug_mode_) {
            ROS_DEBUG("[PnP] Armor %d: dist=%.3fm, pos=[%.3f, %.3f, %.3f]",
                     armor.armor_id, distance,
                     tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        }
        
        return true;
    }
    catch (const cv::Exception &e) {
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

// 简化打印函数
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