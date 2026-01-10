#include "../../include/poseCaculate/poseCaculate.hpp"

// 用于接收图像并缓存
void PoseCaculate::imageCallback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &info_msg) {
    try {
        // 使用锁保护写入过程
        std::lock_guard<std::mutex> lock(img_mutex_);
        // 将ROS图像消息转换为OpenCV格式
        current_image_ = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// 核心绘制函数
void PoseCaculate::drawCoordinateAxis(cv::Mat &img, const cv::Mat &rvec, const cv::Mat &tvec) {
    if (rvec.empty() || tvec.empty() || camera_matrix_.empty() || dist_coeffs_.empty()) return;

    // 1. 定义3D坐标轴的长度（0.1m / 10cm）
    double axis_length = 0.1;

    // 2. 定义世界坐标系中的四个点：原点, X轴端点, Y轴端点, Z轴端点
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));             // Origin
    axisPoints.push_back(cv::Point3f(axis_length, 0, 0));   // X axis
    axisPoints.push_back(cv::Point3f(0, axis_length, 0));   // Y axis
    axisPoints.push_back(cv::Point3f(0, 0, axis_length));   // Z axis

    // 3. 投影到2D图像平面
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, camera_matrix_, dist_coeffs_, imagePoints);

    // 4. 绘制线条
    // 原点到X轴 (红色 - Red)
    cv::line(img, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
    // 原点到Y轴 (绿色 - Green)
    cv::line(img, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
    // 原点到Z轴 (蓝色 - Blue)
    cv::line(img, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
}

