#include "../../include/armorDetect/armorDetect.hpp"

std::vector<cv::Point2f> ArmorDetect::getArmorVertices(const LightDescriptor& left_light, const LightDescriptor& right_light,int armor_type) 
{
    std::vector<cv::Point2f> vertices;
    // 获取左右灯条的四个顶点
    cv::Point2f left_pts[4], right_pts[4];
    left_light.rect.points(left_pts);
    right_light.rect.points(right_pts);
    // 确定装甲板的四个顶点
    // 这里使用的是与 extractFrontImage 函数相同的逻辑
    cv::Point2f src_points[4];
    // 基于灯条方向确定顶点
    // 左侧灯条：取 points[1] 和 points[2]（靠右的两个点）
    // 右侧灯条：取 points[0] 和 points[3]（靠左的两个点）
    src_points[0] = left_pts[1];   // 装甲板左上角
    src_points[1] = right_pts[0];  // 装甲板右上角
    src_points[2] = right_pts[3];  // 装甲板右下角
    src_points[3] = left_pts[2];   // 装甲板左下角
    // 确保顺序正确（左上->右上->右下->左下）
    vertices.push_back(src_points[0]);
    vertices.push_back(src_points[1]);
    vertices.push_back(src_points[2]);
    vertices.push_back(src_points[3]);
    
    return vertices;
}

void ArmorDetect::pubArmorVertices(std::vector<ArmorDescriptor> detected_armors)
{
    armor_detect::ArmorArray armor_array_msg;
    armor_array_msg.header.stamp = ros::Time::now();
    armor_array_msg.header.frame_id = "camera_optical_frame";
    
    // 填充每个装甲板的信息
    for (size_t i = 0; i < detected_armors.size(); i++) {
        const ArmorDescriptor& armor = detected_armors[i];
        
        armor_detect::ArmorInfo armor_info;
        armor_info.header.stamp = ros::Time::now();
        armor_info.header.frame_id = "camera_optical_frame";
        
        armor_info.armor_type = armor.type;
        armor_info.armor_id = i;
        
        // 填充四个顶点坐标
        for (int j = 0; j < 4 && j < armor.vertices.size(); j++) {
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