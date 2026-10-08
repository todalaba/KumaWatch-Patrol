# 熊検知イベント
std_msgs/Header header
bool detected
string bear_id                     # 検知した個体ID(レベル1では既知配置のID)
geometry_msgs/Point robot_position # 検知時のロボット位置(map)
geometry_msgs/Point bear_position  # 熊の推定位置(map)
float64 distance                   # ロボットと熊の距離 [m]
float64 confidence                 # 検知信頼度 [0, 1]
string source                      # "ground_truth_proximity"(レベル1) / "camera_classifier"(レベル2)
