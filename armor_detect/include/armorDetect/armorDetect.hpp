#define WHITE 255,255,255
#define BLACK 0,0,0
#define RED 0,0,255
#define ORANGE 56,119,255
#define YELLOW 0,255,255
#define GREEN 0,255,0
#define BULE 255,0,0
#define PURPLE 231,0,104

#ifndef _ARMOR_DETECT_
#define _ARMOR_DETECT_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <dynamic_reconfigure/server.h>
#include <armor_detect/armorDetectConfig.h>
#include <armor_detect/ArmorArray.h>
#include <armor_detect/ArmorInfo.h>

class ArmorDetect
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;// 订阅图像话题
    image_transport::CameraSubscriber cam_raw_sub_;// 订阅相机图像和信息
    image_transport::Publisher pub_binary_image_;// 发布二值化图像
    image_transport::Publisher pub_result_image_;// 发布绘制结果的图像
    ros::Publisher armor_pub_;// 装甲板发布器
    /* image_transport：
        这个样子发布后还有附加很多子话题：compressed、theora、compressedDepth
        不用管，这是 image_transport 自动创建的，每个子话题里面的内容是不同形式的图片信息
    */
    cv::Mat binary_image_;// 二值化图像

    // 颜色阈值参数
    struct ColorThreshold {
        int hue_min, hue_max;
        int sat_min, sat_max;
        int val_min, val_max;
    } red_thresh_, blue_thresh_;
    int enemy_color_;// 要识别的颜色 0-红色  1-蓝色
    int enemy_type_;// 要识别的装甲板大小 0-small 1-big 2-all

    // 预处理参数
    int brightness_thresh;

    // 形态学操作参数
    struct morphology{
        int morph_open_size;
        int morph_close_size;
        int morph_kernel_size;
    } morphology_;


    // 灯条参数
    struct LightParams {
        double min_area;
        double max_ratio; // 宽高比（宽/高）
        double min_solidity; // 凸度
        double min_height;
    } light_params_;

    // 灯条描述结构体
    struct LightDescriptor {
        cv::RotatedRect rect;  // 旋转矩形
        float length;          // 灯条长度（较长的一边）
        cv::Point2f center;    // 中心点
        float angle;           // 角度（标准化到[-90, 90)度）
        LightDescriptor(const cv::RotatedRect& light_rect) {
            rect = light_rect;
            length = std::max(light_rect.size.width, light_rect.size.height);
            center = light_rect.center;
            
            // 标准化角度到[-90, 90)度
            angle = light_rect.angle;
            if (angle < -90.0) angle += 180.0;
            else if (angle > 90.0) angle -= 180.0;
        }
    };

    struct armor_real {
        int big_width;
        int big_height;
        int small_width;
        int small_height;
    } armor_real_;

    // 装甲板描述结构体
    struct ArmorDescriptor {
        LightDescriptor left_light;
        LightDescriptor right_light;
        cv::Rect bounding_rect;
        int type; // 0-小装甲板  1-大装甲板
        std::vector<cv::Point2f> vertices;// 四个顶点
        ArmorDescriptor(const LightDescriptor& l, const LightDescriptor& r, int armor_type) 
            : left_light(l), right_light(r), type(armor_type) {
            cv::Rect left_rect = l.rect.boundingRect();
            cv::Rect right_rect = r.rect.boundingRect();
            bounding_rect = left_rect | right_rect;
        }
    };

    // 配对参数
    struct PairParams {
        double light_max_angle_diff;
        double light_max_height_diff_ratio;
        double light_max_y_diff_ratio;
        double light_min_x_diff_ratio;
        double armor_min_aspect_ratio;
        double armor_max_aspect_ratio;
        double armor_big_armor_ratio;
        double armor_small_armor_ratio;
    } pair_params_;

    // 数字模板参数
    std::vector<cv::Mat> smallArmorTemplates; // 小装甲板数字模板
    std::vector<cv::Mat> bigArmorTemplates;   // 大装甲板数字模板
    struct digit_resize {
        int big, small;
        double threshold;
    } digit_resize_;

    // 打印参数
    enum get_color {
        white = 0, black, red, orange, yellow, green, bule, purple
    };
    struct draw_detect {
        bool e_light, e_rect, e_l_center, e_r_center, e_label, e_points;
        cv::Scalar e_light_c, e_rect_c, e_l_center_c, e_r_center_c;
    } draw_detect_;

    // 动态参数服务器
    dynamic_reconfigure::Server<armor_detect::armorDetectConfig> dyn_server_;
    dynamic_reconfigure::Server<armor_detect::armorDetectConfig>::CallbackType dyn_callback_;


public:
    ArmorDetect() = default;
    ArmorDetect(ros::NodeHandle& nh);
    ~ArmorDetect() = default;
    // 获取参数
    void initParams();
    cv::Scalar getDrawColor(std::string dst, int default_color);
    // 颜色名称获取
    std::string getColorName(int color_id);
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    // 将图片格式进行转化
    bool convertImage(const sensor_msgs::ImageConstPtr& img_msg, cv::Mat& output_image);
    // 颜色分割
    cv::Mat colorSegmentation(const cv::Mat& hsv_image);
    // 核心检测函数 - 现在返回装甲板向量
    // 建议阅读：https://blog.csdn.net/u010750137/article/details/96428059
    std::vector<ArmorDescriptor> detectArmor(const cv::Mat& image);
    // 绘制检测结果
    void drawDetections(cv::Mat& image, const std::vector<ArmorDescriptor>& armors);
    // 辅助函数：调整旋转矩形方向
    void adjustRect(cv::RotatedRect& rect);
    // 数字模板匹配
    void loadTemplates();
    bool verifyArmorWithTemplate(const cv::Mat& frontImg, int armorType);
    cv::Mat extractFrontImage(const cv::Mat& src, const LightDescriptor& left_light, const LightDescriptor& right_light, int armor_type);

    // 动态参数
    cv::Scalar colorEnumToScalar(int color_enum);
    void dynamicReconfigureCallback(armor_detect::armorDetectConfig &config, uint32_t level);
    
    // 获取顶点、发布
    std::vector<cv::Point2f> selectArmorVertices(const LightDescriptor& left_light, const LightDescriptor& right_light);// 统一的顶点获取接口
    std::vector<cv::Point2f> getArmorVertices(const LightDescriptor& left_light, const LightDescriptor& right_light, int armor_type);
    void pubArmorVertices(std::vector<ArmorDescriptor> detected_armors);
};

#endif