#include "../../include/armorDetect/armorDetect.hpp"
#include <ros/package.h>

ArmorDetect::ArmorDetect(ros::NodeHandle &nh) : nh_(nh), it_(nh)
{
    image_transport::TransportHints hints("compressed");
    cam_raw_sub_ = it_.subscribeCamera("/hk_camera/image_raw", 10, &ArmorDetect::imageCallback, this, hints);
    pub_binary_image_ = it_.advertise("/ArmorDetect/binary_image", 1);
    pub_result_image_ = it_.advertise("/ArmorDetection/result_image", 1);
    armor_pub_ = nh_.advertise<armor_detect::ArmorArray>("/ArmorDetect/armors", 10);
    // 初始化动态参数服务器
    dyn_callback_ = boost::bind(&ArmorDetect::dynamicReconfigureCallback, this, _1, _2);
    dyn_server_.setCallback(dyn_callback_);

    initParams();
    loadTemplates();
}

void ArmorDetect::initParams()
{
    // 红色阈值
    red_thresh_.hue_min = nh_.param("threshold/red/hue_min", 0);
    red_thresh_.hue_max = nh_.param("threshold/red/hue_max", 10);
    red_thresh_.sat_min = nh_.param("threshold/red/sat_min", 100);
    red_thresh_.sat_max = nh_.param("threshold/red/sat_max", 255);
    red_thresh_.val_min = nh_.param("threshold/red/val_min", 50);
    red_thresh_.val_max = nh_.param("threshold/red/val_max", 255);

    // 蓝色阈值
    blue_thresh_.hue_min = nh_.param("threshold/blue/hue_min", 100);
    blue_thresh_.hue_max = nh_.param("threshold/blue/hue_max", 130);
    blue_thresh_.sat_min = nh_.param("threshold/blue/sat_min", 100);
    blue_thresh_.sat_max = nh_.param("threshold/blue/sat_max", 255);
    blue_thresh_.val_min = nh_.param("threshold/blue/val_min", 50);
    blue_thresh_.val_max = nh_.param("threshold/blue/val_max", 255);

    // 敌方颜色、装甲板
    enemy_color_ = nh_.param("enemy_color", 0);
    enemy_type_ = nh_.param("enemy_type", 0);

    // 灯条参数
    light_params_.min_area = nh_.param("light/min_area", 50.0);
    light_params_.max_ratio = nh_.param("light/max_ratio", 0.4);
    light_params_.min_solidity = nh_.param("light/min_solidity", 0.7);
    light_params_.min_height = nh_.param("light/min_height", 20.0);

    // 配对参数
    pair_params_.light_max_angle_diff = nh_.param("pair/light_max_angle_diff", 15.0);
    pair_params_.light_max_height_diff_ratio = nh_.param("pair/light_max_height_diff_ratio", 0.5);
    pair_params_.light_max_y_diff_ratio = nh_.param("pair/light_max_y_diff_ratio", 0.7);
    pair_params_.light_min_x_diff_ratio = nh_.param("pair/light_min_x_diff_ratio", 0.5);
    pair_params_.armor_min_aspect_ratio = nh_.param("pair/armor_min_aspect_ratio", 0.8);
    pair_params_.armor_max_aspect_ratio = nh_.param("pair/armor_max_aspect_ratio", 4.5);
    pair_params_.armor_big_armor_ratio = nh_.param("pair/armor_big_armor_ratio", 3.2);
    pair_params_.armor_small_armor_ratio = nh_.param("pair/armor_small_armor_ratio", 2.0);

    // 预处理参数
    brightness_thresh = nh_.param("preprocess/brightness_thresh", 150);

    // 形态学参数
    morphology_.morph_open_size = nh_.param("morphology/open_size", 3);
    morphology_.morph_close_size = nh_.param("morphology/close_size", 5);
    morphology_.morph_kernel_size = nh_.param("morphology/kernel_size", 4);
    
    // 真实装甲板尺寸参数
    armor_real_.big_width = nh_.param("armor_real/big_width", 138);
    armor_real_.big_height = nh_.param("armor_real/big_height", 75);
    armor_real_.small_width = nh_.param("armor_real/small_width", 108);
    armor_real_.small_height = nh_.param("armor_real/small_height", 100);
    std::cout << "armor_real_: " << armor_real_.big_width << ", " << armor_real_.big_height << ", "
              << armor_real_.small_width << ", " << armor_real_.small_height << std::endl;
    // 模板参数
    digit_resize_.big = nh_.param("digit/big", 28);
    digit_resize_.small = nh_.param("digit/small", 70);
    digit_resize_.threshold = nh_.param("digit/threshold", 0.65);

    // 打印参数
    draw_detect_.e_light = nh_.param("draw_detect/e_light", true);
    draw_detect_.e_rect = nh_.param("draw_detect/e_rect", true);
    draw_detect_.e_l_center = nh_.param("draw_detect/e_l_center", true);
    draw_detect_.e_r_center = nh_.param("draw_detect/e_r_center", true);
    draw_detect_.e_label = nh_.param("draw_detect/e_label", true);
    draw_detect_.e_points = nh_.param("draw_detect/e_points", true);
    
    draw_detect_.e_light_c = getDrawColor("draw_color/e_light", purple);
    draw_detect_.e_rect_c = getDrawColor("draw_color/e_rect", green);
    draw_detect_.e_l_center_c = getDrawColor("draw_color/e_l_center", yellow);
    draw_detect_.e_r_center_c = getDrawColor("draw_color/e_r_center", red);
}

cv::Scalar ArmorDetect::colorEnumToScalar(int color_enum)
{
    switch (color_enum)
    {
    case white:
        return cv::Scalar(WHITE);
    case black:
        return cv::Scalar(BLACK);
    case red:
        return cv::Scalar(RED);
    case orange:
        return cv::Scalar(ORANGE);
    case yellow:
        return cv::Scalar(YELLOW);
    case green:
        return cv::Scalar(GREEN);
    case bule:
        return cv::Scalar(BULE);
    case purple:
        return cv::Scalar(PURPLE);
    default:
        return cv::Scalar(WHITE);
    }
}

cv::Scalar ArmorDetect::getDrawColor(std::string dst, int default_color)
{
    int c = default_color;
    c = nh_.param(dst, default_color);
    return colorEnumToScalar(c);
}

std::string ArmorDetect::getColorName(int color_id)
{
    switch (color_id)
    {
    case 0:
        return "RED";
    case 1:
        return "BLUE";
    default:
        return "UNKNOWN";
    }
}

void ArmorDetect::imageCallback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    cv::Mat image;
    // 1、图片格式转化
    if (!convertImage(img_msg, image))
    {
        ROS_ERROR("failed to convert image");
        return;
    }

    // 2、检测、发布装甲板
    std::vector<ArmorDescriptor> detected_armors = detectArmor(image);
    pubArmorVertices(detected_armors);
    
    // 3、绘制检测结果
    cv::Mat result_image = image.clone();
    drawDetections(result_image, detected_armors);

    // 4、将结果发送到rqt中显示（发布topic）
    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();
    pub_result_image_.publish(result_msg);
    // 附加：二值化图像
    cv::Mat binary_color;

    if (binary_image_.channels() == 1)
        cv::cvtColor(binary_image_, binary_color, cv::COLOR_GRAY2BGR);
    else
        binary_color = binary_image_;

    sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", binary_color).toImageMsg();
    pub_binary_image_.publish(binary_msg);
}

bool ArmorDetect::convertImage(const sensor_msgs::ImageConstPtr &img_msg, cv::Mat &output_image)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        // 转换为BGR8格式
        if (img_msg->encoding == sensor_msgs::image_encodings::BGR8)
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            output_image = cv_ptr->image;
            return true;
        }
        else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
            cv::cvtColor(cv_ptr->image, output_image, cv::COLOR_RGB2BGR);
            return true;
        }
        else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8)
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_ptr->image, output_image, cv::COLOR_GRAY2BGR);
            return true;
        }
        else
        {
            // 尝试使用原始编码
            cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
            ROS_WARN("Unusual encoding: %s", img_msg->encoding.c_str());
            output_image = cv_ptr->image;
            return true;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Image conversion error: %s", e.what());
        return false;
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Processing error: %s", e.what());
        return false;
    }
}

cv::Mat ArmorDetect::colorSegmentation(const cv::Mat &hsv_image)
{
    cv::Mat mask;
    if (enemy_color_ == 0)
    {
        // 检测红色 - 两个范围
        cv::Mat mask1, mask2;
        cv::inRange(hsv_image,
                    cv::Scalar(red_thresh_.hue_min, red_thresh_.sat_min, red_thresh_.val_min),
                    cv::Scalar(red_thresh_.hue_max, red_thresh_.sat_max, red_thresh_.val_max),
                    mask1);
        cv::inRange(hsv_image,
                    cv::Scalar(170, red_thresh_.sat_min, red_thresh_.val_min),
                    cv::Scalar(180, red_thresh_.sat_max, red_thresh_.val_max),
                    mask2);
        mask = mask1 | mask2;
    }
    else
    {
        // 检测蓝色
        cv::inRange(hsv_image,
                    cv::Scalar(blue_thresh_.hue_min, blue_thresh_.sat_min, blue_thresh_.val_min),
                    cv::Scalar(blue_thresh_.hue_max, blue_thresh_.sat_max, blue_thresh_.val_max),
                    mask);
    }
    // 形态学操作：去除噪声
    if (morphology_.morph_open_size > 0)
    {
        cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphology_.morph_open_size, morphology_.morph_open_size));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_open);
    }

    if (morphology_.morph_close_size > 0)
    {
        cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphology_.morph_close_size, morphology_.morph_close_size));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_close);
    }
    return mask;
}
std::vector<ArmorDetect::ArmorDescriptor> ArmorDetect::detectArmor(const cv::Mat &image)
{
    std::vector<ArmorDescriptor> final_armors;
    std::vector<LightDescriptor> light_infos;
    // 红色装甲板：红色通道减蓝色通道，蓝色相反
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    cv::Mat gray_img = (enemy_color_ == 0) ? (channels.at(2) - channels.at(0)) : (channels.at(0) - channels.at(2));
    // 二值化 - 尝试过动态阈值，和预期不一样，画面有点诡异
    cv::Mat bin_img;
    cv::threshold(gray_img, bin_img, brightness_thresh, 255, cv::THRESH_BINARY);
    // 形态学操作
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(morphology_.morph_kernel_size, morphology_.morph_kernel_size));
    cv::dilate(bin_img, bin_img, element);
    // 查找轮廓
    std::vector<std::vector<cv::Point>> light_contours;
    cv::findContours(bin_img.clone(), light_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 筛选灯条轮廓
    for (const auto &contour : light_contours)
    {
        float light_contour_area = cv::contourArea(contour);
        if (light_contour_area < light_params_.min_area)
            continue;
        // min矩形拟合得到旋转矩形 - 原来的是fitEllipse，出现了灯条缺失的情况(不能完整地识别出所有灯条)
        cv::RotatedRect light_rec = cv::minAreaRect(contour);
        adjustRect(light_rec);
        // 宽高比筛选
        float ratio = light_rec.size.width / light_rec.size.height;
        if (ratio > light_params_.max_ratio)
            continue;
        // 凸度筛选
        float solidity = light_contour_area / (light_rec.size.width * light_rec.size.height);
        if (solidity < light_params_.min_solidity)
            continue;
        // 高度筛选
        if (light_rec.size.height < light_params_.min_height)
            continue;
        // 保存灯条
        light_infos.push_back(LightDescriptor(light_rec));
    }
    // 保存二值化图像 P3必做
    binary_image_ = bin_img.clone();
    // 如果没有找到足够灯条，直接返回
    if (light_infos.size() < 2)
    {
        return final_armors;
    }
    // 按灯条中心x坐标排序
    std::sort(light_infos.begin(), light_infos.end(),
              [](const LightDescriptor &ld1, const LightDescriptor &ld2)
              { return ld1.center.x < ld2.center.x; });
    // 装甲板识别：先前的版本中添加了数字模板匹配，但是逻辑有问题：识别装甲板时直接匹配数字，如果匹配数字时报错/崩溃，整个循环就会被破坏掉
    // 参考CSDN，修改如下：
    // ==========【核心修改部分】==========
    // 1. 首先，仅基于几何约束进行灯条配对，生成所有“候选装甲板”
    std::vector<ArmorDescriptor> candidate_armors;
    for (size_t i = 0; i < light_infos.size(); i++)
    {
        for (size_t j = i + 1; j < light_infos.size(); j++)
        {
            const LightDescriptor &left_light = light_infos[i];
            const LightDescriptor &right_light = light_infos[j];
            // 条件1: 角度差（平行度）
            float angle_diff = std::abs(left_light.angle - right_light.angle);
            if (angle_diff > pair_params_.light_max_angle_diff)
                continue;
            // 条件2: 长度差比率
            float len_diff_ratio = std::abs(left_light.length - right_light.length) / std::max(left_light.length, right_light.length);
            if (len_diff_ratio > pair_params_.light_max_height_diff_ratio)
                continue;
            // 条件3: y方向距离
            float y_diff = std::abs(left_light.center.y - right_light.center.y);
            float y_diff_ratio = y_diff / std::max(left_light.length, right_light.length);
            if (y_diff_ratio > pair_params_.light_max_y_diff_ratio)
                continue;
            // 条件4: x方向距离（不应太近）
            float x_diff = std::abs(left_light.center.x - right_light.center.x);
            float x_diff_ratio = x_diff / std::max(left_light.length, right_light.length);
            if (x_diff_ratio < pair_params_.light_min_x_diff_ratio)
                continue;
            // 条件5: 装甲板长宽比
            float distance = cv::norm(left_light.center - right_light.center);
            float mean_len = (left_light.length + right_light.length) / 2;
            float aspect_ratio = distance / mean_len;
            if (aspect_ratio < pair_params_.armor_min_aspect_ratio || aspect_ratio > pair_params_.armor_max_aspect_ratio)
                continue;
            // 确定装甲板类型（大小装甲板）
            int armor_type = (aspect_ratio > pair_params_.armor_big_armor_ratio) ? 1 : 0;
            // ==========================================================================
            // 【修改点】不再在此处立即进行模板验证，先生成候选装甲板
            // cv::Mat frontImg = extractFrontImage(image, left_light, right_light, armor_type);
            // if(!verifyArmorWithTemplate(frontImg, armor_type)) continue;
            // ==========================================================================
            // 添加到候选列表
            if(armor_type == enemy_type_ || enemy_type_ == 2)
            {
                ArmorDescriptor armor(left_light, right_light, armor_type);
                armor.vertices = getArmorVertices(left_light, right_light, armor_type);
                candidate_armors.push_back(armor);
            }
        }
    }
    // 2. 然后，对候选装甲板进行后续验证
    for (auto &armor : candidate_armors)
    {
        cv::Mat frontImg = extractFrontImage(image, armor.left_light, armor.right_light, armor.type);
        int matched_digit = verifyArmorWithTemplate(frontImg, armor);
        if (matched_digit >= 0) 
        {
            armor.number = matched_digit; // 存储匹配到的数字
            final_armors.push_back(armor);
        }
    }
    // ==========【修改结束】==========
    return final_armors;
}

void ArmorDetect::drawDetections(cv::Mat &image, const std::vector<ArmorDescriptor> &armors)
{
    cv::Scalar text_color;
    if (enemy_color_ == 0)
        text_color = cv::Scalar(0, 0, 255);
    else
        text_color = cv::Scalar(0, 165, 255);
    // 绘制检测结果
    for (size_t i = 0; i < armors.size(); i++)
    {
        const ArmorDescriptor &armor = armors[i];

        // 1. 画出左右灯条（紫色）
        if (draw_detect_.e_light)
        {
            cv::ellipse(image, armor.left_light.rect, draw_detect_.e_light_c, 2);
            cv::ellipse(image, armor.right_light.rect, draw_detect_.e_light_c, 2);
        }
        // 2. 画出整个装甲板的外接矩形（绿色）
        if (draw_detect_.e_rect)
        {
            cv::rectangle(image, armor.bounding_rect, draw_detect_.e_rect_c, 3);
        }

        // 3. 连接左右灯条的中心点（黄色）
        if (draw_detect_.e_l_center)
        {
            cv::line(image, armor.left_light.center, armor.right_light.center, draw_detect_.e_l_center_c, 2);
        }
        // 4. 绘制装甲板中心点（红色）
        if (draw_detect_.e_r_center)
        {
            cv::Point armor_center(
                armor.bounding_rect.x + armor.bounding_rect.width / 2,
                armor.bounding_rect.y + armor.bounding_rect.height / 2);
            cv::circle(image, armor_center, 5, draw_detect_.e_r_center_c, -1);
        }


        // 5. 添加标签
        if (draw_detect_.e_label)
        {
            std::string label = "Armor " + std::to_string(i) + (armor.type == 1 ? " (BIG)" : " (SMALL)");
            cv::putText(image, label,
                        cv::Point(armor.bounding_rect.x, armor.bounding_rect.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);
        }
        if(draw_detect_.e_points)
        {
            for (size_t i = 0; i < armors.size(); i++) 
            {
                    const ArmorDescriptor &armor = armors[i];
                    // 用不同颜色和数字标记四个顶点
                    std::vector<cv::Scalar> colors = {
                        cv::Scalar(255, 0, 0),    // 蓝色 - 左上
                        cv::Scalar(0, 255, 0),    // 绿色 - 右上
                        cv::Scalar(0, 0, 255),    // 红色 - 右下
                        cv::Scalar(255, 255, 0)   // 青色 - 左下
                    };
                    std::vector<std::string> labels = {"TL", "TR", "BR", "BL"};
                    for (int j = 0; j < armor.vertices.size() && j < 4; j++) 
                    {
                        // 绘制点
                        cv::circle(image, armor.vertices[j], 5, colors[j], -1);
                        // 添加标签
                        cv::putText(image, labels[j] + "(" + std::to_string(j) + ")",
                                cv::Point(armor.vertices[j].x + 10, armor.vertices[j].y - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, colors[j], 2);
                    }
            }
        }
    }

    // 在图像左上角添加统计信息
    std::string info_text = "Detected: " + std::to_string(armors.size()) + " armors";
    cv::putText(image, info_text,
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);

    // 添加颜色信息
    std::string color_text = "Target: " + getColorName(enemy_color_);
    cv::putText(image, color_text,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);
}

void ArmorDetect::adjustRect(cv::RotatedRect &rect)
{
    // 确保宽度是较小的边（灯条宽度）
    if (rect.size.width > rect.size.height)
    {
        std::swap(rect.size.width, rect.size.height);
        rect.angle += 90.0f;
    }

    // 标准化角度到[-90, 90)度
    if (rect.angle < -90.0)
        rect.angle += 180.0;
    else if (rect.angle > 90.0)
        rect.angle -= 180.0;
}

// ============================================================
void ArmorDetect::loadTemplates()
{
    smallArmorTemplates.clear();
    bigArmorTemplates.clear();

    ROS_INFO("[Template] Starting to load armor digit templates...");

    std::string package_path;
    try
    {
        package_path = ros::package::getPath("armor_detect");
        if (package_path.empty())
        {
            ROS_ERROR("[Template] Failed to get package path!");
            return;
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Template] Exception when getting package path: %s", e.what());
        return;
    }

    std::string template_dir = package_path + "/Template/";
    ROS_INFO("[Template] Looking for templates in: %s", template_dir.c_str());

    int loaded_count = 0;
    for (int digit = 1; digit <= 8; ++digit)
    {
        std::string small_file = template_dir + std::to_string(digit) + std::to_string(digit) + ".jpg";
        std::string big_file = template_dir + std::to_string(digit) + ".jpg";

        cv::Mat small_tpl = cv::imread(small_file, cv::IMREAD_GRAYSCALE);
        cv::Mat big_tpl = cv::imread(big_file, cv::IMREAD_GRAYSCALE);

        // 检查是否成功加载
        if (small_tpl.empty() || big_tpl.empty())
        {
            ROS_WARN("[Template] Could not load template(s) for digit %d (files: %s, %s). Skipping.",
                     digit, small_file.c_str(), big_file.c_str());
            continue; // 跳过这个数字，继续加载下一个
        }
        // 可选：调整大小至统一尺寸 - 不建议使用，使用原图片尺寸进行匹配的效果更好
        //cv::resize(small_tpl, small_tpl, cv::Size(digit_resize_.small, digit_resize_.small));
        // cv::resize(big_tpl, big_tpl, cv::Size(digit_resize_.big, digit_resize_.big));
        // 确保图像类型为 CV_8UC1（单通道8位），为后续 matchTemplate 做准备
        if (small_tpl.type() != CV_8UC1) small_tpl.convertTo(small_tpl, CV_8UC1);
        if (big_tpl.type() != CV_8UC1) big_tpl.convertTo(big_tpl, CV_8UC1);

        smallArmorTemplates.push_back({digit, small_tpl});
        bigArmorTemplates.push_back({digit, big_tpl});
        loaded_count++;

        ROS_DEBUG("[Template] Successfully loaded template for digit %d", digit);
    }
    if (loaded_count > 0)
    {
        ROS_INFO("[Template] Template loading COMPLETE. Loaded %d digit(s).", loaded_count);
        ROS_INFO("[Template]   Small armor templates: %ld", smallArmorTemplates.size());
        ROS_INFO("[Template]   Big armor templates: %ld", bigArmorTemplates.size());
    }
    else
    {
        ROS_WARN("[Template] WARNING: No valid digit templates were loaded.");
        ROS_WARN("[Template] Program will run WITHOUT digit recognition (armor verification).");
    }
}

int ArmorDetect::verifyArmorWithTemplate(const cv::Mat &frontImg, ArmorDescriptor armor)
{
    const std::vector<TemplateMatch> &templates = (armor.type == 0) ? smallArmorTemplates : bigArmorTemplates;
    double maxScore = 0;
    // 将正面图像转为灰度并缩放到与模板相同尺寸
    cv::Mat grayFront;
    // 检查输入图像通道数
    if (frontImg.channels() == 3)
    {
        // 如果是彩色图，转换为灰度图
        cvtColor(frontImg, grayFront, cv::COLOR_BGR2GRAY);
        ROS_DEBUG("convert color image to grayscale");
    }
    else if (frontImg.channels() == 1)
    {
        // 如果已经是灰度图，直接使用
        grayFront = frontImg.clone();
        ROS_DEBUG("already grayscale, use directly");
    }
    else
    {
        ROS_ERROR("verifyArmorWithTemplate: Unexpected number of channels: %d", frontImg.channels());
        return false;
    }

    // 确保图像深度为 CV_8U
    if (grayFront.depth() != CV_8U)
    {
        ROS_WARN("Converting image depth from %d to CV_8U", grayFront.depth());
        grayFront.convertTo(grayFront, CV_8U);
    }

    // 与所有模板进行匹配，取最高分
    int best_id = 0;
    double max_val = -1;
    for (const auto& temp : templates) 
    {
        cv::Mat result;
        cv::matchTemplate(frontImg, temp.image, result, cv::TM_CCOEFF_NORMED);
        double min_val, current_max_val;
        cv::minMaxLoc(result, &min_val, &current_max_val);
        if (current_max_val > max_val) 
        {
            max_val = current_max_val;
            best_id = temp.number;
        }
    }
    // 判断最大匹配分数是否超过阈值
    if (max_val < digit_resize_.threshold) return -1;
    // 判断阈值，例如大于阈值则认为匹配成功（有数字）
    return best_id;
}

cv::Mat ArmorDetect::extractFrontImage(const cv::Mat &src, const LightDescriptor &left_light, const LightDescriptor &right_light, int armor_type)
{
    // 检查输入图像
    if (src.empty())
    {
        ROS_WARN("extractFrontImage: Empty source image");
        return cv::Mat();
    }
    // 检查通道数
    if (src.channels() == 1)
    {
        ROS_WARN("extractFrontImage: Source image is grayscale (1 channel)");
        // 转换为彩色图
        cv::Mat color_src;
        cv::cvtColor(src, color_src, cv::COLOR_GRAY2BGR);
        return extractFrontImage(color_src, left_light, right_light, armor_type);
    }
    if (src.channels() != 3)
    {
        ROS_ERROR("extractFrontImage: Unexpected number of channels: %d", src.channels());
        return cv::Mat();
    }

    // 1. 使用统一的顶点选择函数
    std::vector<cv::Point2f> src_points = selectArmorVertices(left_light, right_light);
    
    if (src_points.size() != 4)
    {
        ROS_WARN("extractFrontImage: Failed to get 4 vertices, got %zu", src_points.size());
        return cv::Mat();
    }
    
    // 2. 定义目标图像尺寸
    int width, height;
    if (armor_type == 1)  // 大装甲板
    {                
        width = armor_real_.big_width ;
        height = armor_real_.big_height; 
    }
    else  // 小装甲板
    { 
        width = armor_real_.small_width;
        height = armor_real_.small_height;
    }
    
    // 3. 目标图像的四个角点
    std::vector<cv::Point2f> dst_points;
    dst_points.push_back(cv::Point2f(0, 0));           // 左上
    dst_points.push_back(cv::Point2f(width, 0));       // 右上
    dst_points.push_back(cv::Point2f(width, height));  // 右下
    dst_points.push_back(cv::Point2f(0, height));      // 左下
    
    // 4. 计算透视变换并执行
    cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);
    cv::Mat front_img;
    cv::warpPerspective(src, front_img, perspective_matrix, cv::Size(width, height));
    
    // 5. 转换为灰度图
    if (front_img.channels() == 3)
    {
        cv::cvtColor(front_img, front_img, cv::COLOR_BGR2GRAY);
    }

    return front_img;
}