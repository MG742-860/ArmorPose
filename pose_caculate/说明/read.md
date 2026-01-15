# 装甲板坐标轴滞后
	观察到：函数并**不在**装甲板中心绘制坐标轴。事实上，坐标轴有过渡动画，在rqt中显示有些许延迟，当持续播放图像信息，或是摄像机一直工作，会让人产生坐标轴相对于装甲板而言是落后的，比如装甲板向左移动，那么坐标轴在装甲板中心靠右的位置，当暂停时，图像（装甲板）停止运动，坐标轴此时现实在装甲板中心，是预期的、理想的位置。初步观察，应该是因为有坐标轴动画过渡导致的。	
### 代码分析
是滤波导致的：
```cpp
if (last_tvecs.find(id) != last_tvecs.end())
{
    double move_dist = cv::norm(tvec - last_tvecs[id]);

    if (move_dist < 1.0)
    { // 正常运动范围
        // --- 自适应 Alpha ---
        // 距离 2m 时 alpha 约 0.7 (较灵敏)
        // 距离 4m 时 alpha 约 0.3 (极度平滑)
        double alpha = 1.0 - (current_distance / 6.0);
        alpha = std::max(0.2, std::min(0.8, alpha)); // 限制范围

        // 如果是大装甲板，额外增强平滑度
        if (armor.armor_type == 1)
        {
            alpha *= 0.7;
        }

        tvec = alpha * tvec + (1.0 - alpha) * last_tvecs[id];
        rvec = alpha * rvec + (1.0 - alpha) * last_rvecs[id];
    }
    // 如果 move_dist 过大，说明是新目标或跳变，直接更新不滤波
}
```
**应用滤波后，当前：新的位姿 = alpha * 当前解算值 + (1-alpha) * 上一次的历史值 **
这就导致了观察到的坐标轴看上去像是有过渡动画一样，事实是应用了滤波导致上一帧所占有的比值比较大，观察到坐标轴滞后。
### 解决方法
**1、更改绘制逻辑**
**2、直接删除滤波**

    由于第二种没有什么好说明的，只需要delete/unable就行了，下面讲解第一种（注意：删除滤波后你可能在rviz中看到装甲板坐标轴小幅度抽搐，不过在允许范围内，它的总体运动轨迹和角度在理想范围内）
首先再新定义一个函数，用于在获取到实时的tvec和rvec后可以进行坐标轴绘制：
```cpp
void PoseCaculate::pub_debug_image(const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs)
{
    cv::Mat debug_image;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (current_image_.empty()) return;
        debug_image = current_image_.clone(); // 只拷贝一次底图
    }

    // 检查数据一致性
    if (rvecs.size() != tvecs.size()) return;

    for (size_t i = 0; i < rvecs.size(); i++)
    {
        // 调用单次绘制函数
        drawCoordinateAxis(debug_image, rvecs[i], tvecs[i]);
    }

    // 发布最终合成的图像
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
    out_msg->header.stamp = ros::Time::now();
    image_pub_.publish(out_msg);
}
// 原来的坐标轴绘制函数，用于单词绘制，此处没有修改，只是搬用进行说明。
void PoseCaculate::drawCoordinateAxis(cv::Mat &img, const cv::Mat &rvec, const cv::Mat &tvec)
{
    if (rvec.empty() || tvec.empty() || camera_matrix_.empty() || dist_coeffs_.empty())
        return;

    // 1. 定义3D坐标轴的长度（0.1m / 10cm）
    double axis_length = 0.1;

    // 2. 定义世界坐标系中的四个点：原点, X轴端点, Y轴端点, Z轴端点
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));           // Origin
    axisPoints.push_back(cv::Point3f(axis_length, 0, 0)); // X axis
    axisPoints.push_back(cv::Point3f(0, axis_length, 0)); // Y axis
    axisPoints.push_back(cv::Point3f(0, 0, axis_length)); // Z axis

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
```

然后更改SolvePnp函数进行存储原始数据集：

```cpp
bool solvePnPForArmor(const armor_detect::ArmorInfo &armor,cv::Mat &rvec, cv::Mat &tvec);
{
    // 1. 提取2D像素点
        ...
    // 2. 获取3D模型点
        ...
    // 3. 检查2D点面积
    ...
    // 4. 调用solvePnP
        ...
        cv::Mat filter_rvec = rvec.clone();// success后直接复制
        cv::Mat filter_tvec = tvec.clone();
    // 后面用的全部是filter_rvec、filter_tvec。
    // 5. 检查距离合理性
        ...
    // 6. 验证位姿
        ...
    // 7. 位姿滤波
        ...
        ...
}
```

然后在原来的armorCallback函数中进行绘制：
```cpp
    if (axis_drawn_ )
    {
        pub_debug_image(rvecs_list, tvecs_list);
    }
```

*** 
现在就解决了debug_image图像中坐标轴滞后的问题了。

	现在仍然存在滞后性：绘制all时能明显感觉到滞后，但是当你pause后又能看到坐标轴移动回去装甲板中心，现在解决了滞后感很强的问题，但是没有彻底解决。
# 附加修改（额外功能）


