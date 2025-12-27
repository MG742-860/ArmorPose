#include "../../include/armorDetect/armorDetect.hpp"



// 动态参数回调函数实现
void ArmorDetect::dynamicReconfigureCallback(armor_detect::armorDetectConfig &config, uint32_t level)
{
    ROS_INFO("[Dynamic Reconfigure] Updating parameters...");
    
    // 更新敌方颜色
    enemy_color_ = config.enemy_color;
    enemy_type_ = config.enemy_type;

    // 更新红色阈值
    red_thresh_.hue_min = config.red_hue_min;
    red_thresh_.hue_max = config.red_hue_max;
    red_thresh_.sat_min = config.red_sat_min;
    red_thresh_.sat_max = config.red_sat_max;
    red_thresh_.val_min = config.red_val_min;
    red_thresh_.val_max = config.red_val_max;
    
    // 更新蓝色阈值
    blue_thresh_.hue_min = config.blue_hue_min;
    blue_thresh_.hue_max = config.blue_hue_max;
    blue_thresh_.sat_min = config.blue_sat_min;
    blue_thresh_.sat_max = config.blue_sat_max;
    blue_thresh_.val_min = config.blue_val_min;
    blue_thresh_.val_max = config.blue_val_max;
    
    // 更新灯条参数
    light_params_.min_area = config.light_min_area;
    light_params_.max_ratio = config.light_max_ratio;
    light_params_.min_solidity = config.light_min_solidity;
    light_params_.min_height = config.light_min_height;
    
    // 更新配对参数
    pair_params_.light_max_angle_diff = config.light_max_angle_diff;
    pair_params_.light_max_height_diff_ratio = config.light_max_height_diff_ratio;
    pair_params_.light_max_y_diff_ratio = config.light_max_y_diff_ratio;
    pair_params_.light_min_x_diff_ratio = config.light_min_x_diff_ratio;
    pair_params_.armor_min_aspect_ratio = config.armor_min_aspect_ratio;
    pair_params_.armor_max_aspect_ratio = config.armor_max_aspect_ratio;
    pair_params_.armor_big_armor_ratio = config.armor_big_armor_ratio;
    pair_params_.armor_small_armor_ratio = config.armor_small_armor_ratio;
    
    // 更新预处理参数
    brightness_thresh = config.brightness_thresh;
    
    // 更新形态学参数
    morphology_.morph_open_size = config.morph_open_size;
    morphology_.morph_close_size = config.morph_close_size;
    morphology_.morph_kernel_size = config.morph_kernel_size;
    
    // 更新数字识别参数
    digit_resize_.big = config.digit_big;
    digit_resize_.small = config.digit_small;
    digit_resize_.threshold = config.digit_threshold;
    
    // 更新绘制开关
    draw_detect_.e_light = config.draw_e_light;
    draw_detect_.e_rect = config.draw_e_rect;
    draw_detect_.e_l_center = config.draw_e_l_center;
    draw_detect_.e_r_center = config.draw_e_r_center;
    draw_detect_.e_label = config.draw_e_label;
    draw_detect_.e_points = config.draw_e_points;
    
    // 更新绘制颜色
    draw_detect_.e_light_c = colorEnumToScalar(config.draw_color_e_light);
    draw_detect_.e_rect_c = colorEnumToScalar(config.draw_color_e_rect);
    draw_detect_.e_l_center_c = colorEnumToScalar(config.draw_color_e_l_center);
    draw_detect_.e_r_center_c = colorEnumToScalar(config.draw_color_e_r_center);
    
    ROS_INFO("[Dynamic Reconfigure] Parameters updated successfully!");
    ROS_INFO("[Dynamic Reconfigure] Enemy color: %s", getColorName(enemy_color_).c_str());
}