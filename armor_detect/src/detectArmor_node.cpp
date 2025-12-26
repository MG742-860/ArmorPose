#include "armorDetect/armorDetect.hpp"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "armor_detection_node");
    ros::NodeHandle nh;
    // 设置ROS日志级别
    if (ros::console::set_logger_level
        (ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ROS_INFO("Starting Armor Detector Node");
    ROS_INFO("check at rqt: result_image/binary_image");
    
    ArmorDetect node(nh);
    
    ros::spin();
    
    ROS_INFO("Armor Detector Node shutting down");
    return 0;
}