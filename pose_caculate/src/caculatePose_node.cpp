#include "../include/poseCaculate/poseCaculate.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pose_caculate_node");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 设置日志级别（可选）
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                                       ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ROS_INFO("Starting pose_caculate_node...");
    
    try
    {
        // 创建PoseCaculate对象（所有功能都在构造函数中初始化）
        PoseCaculate pose_caculate(nh);
        
        ROS_INFO("pose_caculate_node is running...");
        ROS_INFO("Press Ctrl+C to exit");
        
        // 进入ROS主循环
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception in pose_caculate_node: %s", e.what());
        return 1;
    }
    
    return 0;
}