import numpy as np
from scipy.spatial.transform import Rotation as R

def solve():
    # 1. 前方LiDARの真値 (base_linkから見た位置)
    # x=0.4, y=0.0, z=0.7, rotation=none
    t_base_front = np.eye(4)
    t_base_front[:3, 3] = [0.4, 0.0, 0.7]

    # 2. プログラム1からコピーした行列 (例として仮の値を入れます)
    mat_bridge_front = np.array([[ 1.0,  0.0,  0.0, -0.11219622],
    [ 0.0,  1.0,  0.0,  0.00673950],
    [ 0.0,  0.0,  1.0, -0.60805972],
    [ 0.0,  0.0,  0.0,  1.0       ]]) # プログラム1の結果
    mat_bridge_back = np.array([[-1.0,  0.0,  0.0, -0.79889046],
    [ 0.0, -1.0,  0.0, -0.04092905],
    [ 0.0,  0.0,  1.0, -0.69292602],
    [ 0.0,  0.0,  0.0,  1.0       ]]) # プログラム1の結果

    # 3. 計算ロジック
    # ブリッジがどこにいたかを逆算: T_base_bridge = T_base_front * inv(T_bridge_front)
    t_base_bridge = t_base_front @ np.linalg.inv(mat_bridge_front)
    
    # 後方の最終位置を算出: T_base_back = T_base_bridge * T_bridge_back
    t_base_back = t_base_bridge @ mat_bridge_back

    # 4. 結果表示
    x, y, z = t_base_back[:3, 3]
    quat = R.from_matrix(t_base_back[:3, :3]).as_quat()
    
    print("\n=== Final Back LiDAR Position (base_link基準) ===")
    print(f"Arguments: ['{x:.4f}', '{y:.4f}', '{z:.4f}', '{quat[0]:.4f}', '{quat[1]:.4f}', '{quat[2]:.4f}', '{quat[3]:.4f}']")

if __name__ == "__main__": solve()