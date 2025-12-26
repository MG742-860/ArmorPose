# 装甲板信息
Header header

# 装甲板类型：0-小装甲板，1-大装甲板
uint8 armor_type

# 四个顶点的像素坐标（顺序：左上->右上->右下->左下）
geometry_msgs/Point[4] vertices_pixel

# 灯条中心点
geometry_msgs/Point left_light_center
geometry_msgs/Point right_light_center

# 边界框
int32 rect_x
int32 rect_y
int32 rect_width
int32 rect_height

# 装甲板ID（用于跟踪）
uint8 armor_id