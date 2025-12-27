#include "../../include/armorDetect/armorDetect.hpp"

std::vector<cv::Point2f> ArmorDetect::selectArmorVertices(const LightDescriptor& left_light, const LightDescriptor& right_light) 
{
    std::vector<cv::Point2f> vertices;
    cv::Point2f left_pts[4], right_pts[4];
    left_light.rect.points(left_pts);
    right_light.rect.points(right_pts);
    
    // 步骤1：将点存储到向量中
    std::vector<cv::Point2f> left_points(left_pts, left_pts + 4);
    std::vector<cv::Point2f> right_points(right_pts, right_pts + 4);
    
    // 步骤2：分组函数 - 将一个灯条的4个点分成顶部和底部两组
    auto groupPoints = [](const std::vector<cv::Point2f>& points) -> std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> 
    {
        // 计算每个点与其他三个点的距离
        std::vector<std::vector<std::pair<int, float>>> distances(points.size());
        
        for (int i = 0; i < points.size(); i++) {
            for (int j = 0; j < points.size(); j++) 
            {
                if (i != j) 
                {
                    float dist = cv::norm(points[i] - points[j]);
                    distances[i].push_back({j, dist});
                }
            }
            // 按距离排序
            std::sort(distances[i].begin(), distances[i].end(),
                     [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                         return a.second < b.second;});
        }
        
        // 找出最近的两个点对
        std::vector<bool> used(4, false);
        std::vector<std::pair<int, int>> pairs;
        
        for (int i = 0; i < 4; i++) {
            if (!used[i]) 
            {
                // 找到最近的邻居
                int nearest_idx = distances[i][0].first;
                if (!used[nearest_idx]) 
                {
                    pairs.push_back({i, nearest_idx});
                    used[i] = true;
                    used[nearest_idx] = true;
                }
            }
        }
        
        // 应该有两对点
        if (pairs.size() != 2) {
            ROS_ERROR("Failed to group points into 2 pairs, got %zu", pairs.size());
            // 回退方案：按y坐标分两组
            std::vector<cv::Point2f> sorted_points = points;
            std::sort(sorted_points.begin(), sorted_points.end(),
                     [](const cv::Point2f& a, const cv::Point2f& b) {
                         return a.y < b.y;});
            return {{sorted_points[0], sorted_points[1]}, {sorted_points[2], sorted_points[3]}};
        }
        
        // 将点对转换为两组点
        std::vector<cv::Point2f> group1 = {points[pairs[0].first], points[pairs[0].second]};
        std::vector<cv::Point2f> group2 = {points[pairs[1].first], points[pairs[1].second]};
        
        // 按y坐标确定哪组是顶部，哪组是底部
        float group1_avg_y = (group1[0].y + group1[1].y) / 2;
        float group2_avg_y = (group2[0].y + group2[1].y) / 2;
        
        if (group1_avg_y < group2_avg_y) {
            return {group1, group2};  // group1是顶部，group2是底部
        } else {
            return {group2, group1};  // group2是顶部，group1是底部
        }
    };
    
    // 步骤3：对左右灯条进行分组
    auto [left_top_group, left_bottom_group] = groupPoints(left_points);
    auto [right_top_group, right_bottom_group] = groupPoints(right_points);
    
    // 步骤4：在每组中选择距离另一个灯条最近的点
    
    // 对于左边灯条：
    // 顶部组：选择距离右边灯条最近的点
    cv::Point2f left_top_point;
    float min_dist = std::numeric_limits<float>::max();
    for (const auto& pt : left_top_group) {
        float dist = cv::norm(pt - right_light.center);
        if (dist < min_dist) {
            min_dist = dist;
            left_top_point = pt;
        }
    }
    
    // 底部组：选择距离右边灯条最近的点
    cv::Point2f left_bottom_point;
    min_dist = std::numeric_limits<float>::max();
    for (const auto& pt : left_bottom_group) {
        float dist = cv::norm(pt - right_light.center);
        if (dist < min_dist) {
            min_dist = dist;
            left_bottom_point = pt;
        }
    }
    
    // 对于右边灯条：
    // 顶部组：选择距离左边灯条最近的点
    cv::Point2f right_top_point;
    min_dist = std::numeric_limits<float>::max();
    for (const auto& pt : right_top_group) 
    {
        float dist = cv::norm(pt - left_light.center);
        if (dist < min_dist) 
        {
            min_dist = dist;
            right_top_point = pt;
        }
    }
    
    // 底部组：选择距离左边灯条最近的点
    cv::Point2f right_bottom_point;
    min_dist = std::numeric_limits<float>::max();
    for (const auto& pt : right_bottom_group) 
    {
        float dist = cv::norm(pt - left_light.center);
        if (dist < min_dist) 
        {
            min_dist = dist;
            right_bottom_point = pt;
        }
    }
    
    // 步骤5：按顺序排列顶点
    // 注意：有时左边灯条可能在右边灯条的右侧（如果检测顺序错了）
    // 所以我们需要根据x坐标来确定真正的左右
    
    std::vector<cv::Point2f> sorted_points = {left_top_point, left_bottom_point, right_top_point, right_bottom_point};
    
    // 找出x坐标最小的两个点（左边灯条的点）
    std::sort(sorted_points.begin(), sorted_points.end(),
             [](const cv::Point2f& a, const cv::Point2f& b) {return a.x < b.x;});
    
    cv::Point2f left1 = sorted_points[0];
    cv::Point2f left2 = sorted_points[1];
    cv::Point2f right1 = sorted_points[2];
    cv::Point2f right2 = sorted_points[3];
    
    // 在左边两个点中，y小的为上，y大的为下
    cv::Point2f left_top, left_bottom;
    if (left1.y < left2.y) 
    {
        left_top = left1;
        left_bottom = left2;
    } 
    else 
    {
        left_top = left2;
        left_bottom = left1;
    }
    
    // 在右边两个点中，y小的为上，y大的为下
    cv::Point2f right_top, right_bottom;
    if (right1.y < right2.y) 
    {
        right_top = right1;
        right_bottom = right2;
    } 
    else 
    {
        right_top = right2;
        right_bottom = right1;
    }
    
    // 最终顶点顺序：左上 → 右上 → 右下 → 左下
    vertices.push_back(left_top);     // 左上
    vertices.push_back(right_top);    // 右上
    vertices.push_back(right_bottom); // 右下
    vertices.push_back(left_bottom);  // 左下
    
    return vertices;
}

std::vector<cv::Point2f> ArmorDetect::getArmorVertices(const LightDescriptor& left_light, const LightDescriptor& right_light, int armor_type) 
{
    return selectArmorVertices(left_light, right_light);
}

void ArmorDetect::pubArmorVertices(std::vector<ArmorDescriptor> detected_armors)
{
    armor_detect::ArmorArray armor_array_msg;
    armor_array_msg.header.stamp = ros::Time::now();
    armor_array_msg.header.frame_id = "camera_optical_frame";
    
    // 填充每个装甲板的信息
    for (size_t i = 0; i < detected_armors.size(); i++) 
    {
        const ArmorDescriptor& armor = detected_armors[i];
        
        armor_detect::ArmorInfo armor_info;
        armor_info.header.stamp = ros::Time::now();
        armor_info.header.frame_id = "camera_optical_frame";
        
        armor_info.armor_type = armor.type;
        armor_info.armor_id = i;
        
        // 填充四个顶点坐标
        for (int j = 0; j < 4 && j < armor.vertices.size(); j++) 
        {
            geometry_msgs::Point point;
            point.x = armor.vertices[j].x;
            point.y = armor.vertices[j].y;
            point.z = 0.0;  // 2D坐标，z=0
            armor_info.vertices_pixel[j] = point;
        }
        
        // 填充灯条中心点
        geometry_msgs::Point left_center, right_center;
        left_center.x = armor.left_light.center.x;
        left_center.y = armor.left_light.center.y;
        left_center.z = 0.0;
        
        right_center.x = armor.right_light.center.x;
        right_center.y = armor.right_light.center.y;
        right_center.z = 0.0;
        
        armor_info.left_light_center = left_center;
        armor_info.right_light_center = right_center;
        
        // 填充边界框
        armor_info.rect_x = armor.bounding_rect.x;
        armor_info.rect_y = armor.bounding_rect.y;
        armor_info.rect_width = armor.bounding_rect.width;
        armor_info.rect_height = armor.bounding_rect.height;
        
        // 添加到数组
        armor_array_msg.armors.push_back(armor_info);
    }
    
    // 发布装甲板信息
    armor_pub_.publish(armor_array_msg);
}