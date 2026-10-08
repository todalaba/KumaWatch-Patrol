# 1つの巡回候補地点に対するAIの熊出没リスク推定値
string point_id            # 例: "P4"
string name                # 表示名(例: "ごみ集積所")
string category            # residential / forest_edge / school_route / garbage / river
geometry_msgs/Pose pose    # map 座標系での巡回目標姿勢
float64 risk               # 熊出没確率の推定値 [0, 1]
float64 importance_weight  # 社会的重要度の重み w_i (通学路などで > 1)
