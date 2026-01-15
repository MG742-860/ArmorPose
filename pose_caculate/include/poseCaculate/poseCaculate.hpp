/*
* 先获取消息：armor信息和相机内参信息
* 然后对每个装甲板调用solvePnP解算位姿
*/
#ifndef POSE_CACULATE_HPP
#define POSE_CACULATE_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// TF2相关头文件
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <armor_detect/ArmorArray.h>
#include <armor_detect/ArmorInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <mutex>
class PoseCaculate
{
private:
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber armor_sub_;
    ros::Publisher pose_pub_;
    
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_set_ = false;
    
    // 装甲板物理尺寸（单位：米）
    double small_armor_width_;
    double small_armor_height_;
    double big_armor_width_;
    double big_armor_height_;
    
    // 3D模型点
    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> big_armor_points_;
    
    // PnP配置
    int pnp_method_;
    bool enable_filter_;
    
    // 调试和验证
    bool debug_mode_;
    bool print_results_;
    double min_valid_distance_;
    double max_valid_distance_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool publish_tf_;
    bool publish_pose_messages_;
    std::string parent_frame_id_;
    std::string child_frame_prefix_;
    double tf_cache_time_;

    // TF清理
    ros::Timer tf_cleanup_timer_;
    void cleanupOldTf(const ros::TimerEvent& event);
    std::set<int> active_armor_ids_;
    ros::Time last_tf_time_;

    // 可选：原图绘制坐标轴
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat current_image_; // 缓存当前图像
    std::mutex img_mutex_;
    bool axis_drawn_ = false;

public:
    // 构造函数 析构函数
    PoseCaculate(ros::NodeHandle &nh);
    ~PoseCaculate() = default;
    // 参数加载
    void loadParameters();
    //相机信息回调函数
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg); 
    // 3D模型点生成
    void generate3DPoints();
    // 获取对应装甲板类型的3D模型点
    std::vector<cv::Point3f> get3DObjectPoints(int armor_type);

    // ====================================================================================

    // 装甲板消息处理
    void armorCallback(const armor_detect::ArmorArrayConstPtr &armor_msg);
    // PnP解算核心函数
    bool solvePnPForArmor(const armor_detect::ArmorInfo &armor,cv::Mat &rvec, cv::Mat &tvec);
    // 从装甲板消息中提取2D像素点
    bool extractImagePoints(const armor_detect::ArmorInfo &armor,std::vector<cv::Point2f> &image_points);
    // 验证位姿结果的合理性
    bool validatePoseResult(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id);
    // 结果输出 
    void printPoseResult(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id, int armor_type); 

    // ====================================================================================
    
    // 将旋转向量转换为四元数
    tf2::Quaternion rvecToQuaternion(const cv::Mat &rvec);
    // 发布TF变换
    void publishTfTransform(const cv::Mat &rvec, const cv::Mat &tvec,const ros::Time &stamp, int armor_id, int armor_type);
    // 发布Pose消息
    void publishPoseMessage(const cv::Mat &rvec, const cv::Mat &tvec,const ros::Time &stamp, int armor_id, int armor_type);
    void printRotationInfo(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id);
    void validateCoordinateSystem(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id);
    void debugTransform(const cv::Mat &rvec, const cv::Mat &tvec, int armor_id);    

    // 绘制坐标轴到图像上（可选）
    void pub_debug_image(const cv::Mat &rvecs, const cv::Mat &tvecs);
    void drawCoordinateAxis(cv::Mat &img, const cv::Mat &rvec, const cv::Mat &tvec);
    void imageCallback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
};

#endif // POSE_CACULATE_HPP