# RelativePosition.msg

int32 marker_id
float64 x
float64 y
float64 z
float64 distance
float64 angle  # 상대 방향 벡터의 각도 (라디안 단위)
geometry_msgs/Quaternion quaternion
float64 rx  # 롤 (라디안 단위)
float64 ry  # 피치 (라디안 단위)
float64 rz  # 요 (라디안 단위)
