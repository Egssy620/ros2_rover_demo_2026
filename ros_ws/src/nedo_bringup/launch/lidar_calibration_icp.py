import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data

# 変換行列を作成するヘルパー関数
def make_tf(x, y, z, yaw=0.0):
    mat = np.eye(4)
    mat[:3, :3] = R.from_euler('z', yaw).as_matrix()
    mat[:3, 3] = [x, y, z]
    return mat

class BridgeMultiICP(Node):
    def __init__(self):
        super().__init__('bridge_multi_icp')
        self.bridge_topic = '/ouster/points'
        self.front_topic = '/livox/lidar_3JEDM4V001DY881'
        self.back_topic = '/livox/lidar_3JEDM4R0014V101'
        #self.back_topic = '/livox/lidar_3JEDM4V001DY881'
        #self.front_topic = '/livox/lidar_3JEDM4R0014V101'

        self.max_frames = 50
        self.data = {'bridge': [], 'front': [], 'back': []}
        
        self.create_subscription(PointCloud2, self.bridge_topic, lambda m: self.cb(m, 'bridge'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.front_topic, lambda m: self.cb(m, 'front'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.back_topic, lambda m: self.cb(m, 'back'), qos_profile_sensor_data)
        self.get_logger().info("データ蓄積中... 各LiDARの点群を50フレーム取得します。")

    def cb(self, msg, key):
        if len(self.data[key]) < self.max_frames:
            pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pnts = np.array([[p[0], p[1], p[2]] for p in pts], dtype=float)
            self.data[key].append(pnts)
            self.get_logger().info(
                f"取得フレーム数 bridge/front/back = "
                f"{len(self.data['bridge'])}/{len(self.data['front'])}/{len(self.data['back'])}"
            )
            if all(len(v) == self.max_frames for v in self.data.values()):
                self.run_calibration()

    def run_calibration(self):
        self.get_logger().info("3点同時表示によるキャリブレーション確認を開始します...")
        pcds = {}
        import copy
        for k, v in self.data.items():
            pcds[k] = o3d.geometry.PointCloud()
            pcds[k].points = o3d.utility.Vector3dVector(np.vstack(v))
            pcds[k] = pcds[k].voxel_down_sample(0.02)
            cl, ind = pcds[k].remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            pcds[k] = pcds[k].select_by_index(ind)
            pcds[k].estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # 1. 各LiDARの初期位置（設計値）
        t_base_front_true  = make_tf(0.4, 0.0, 0.7, yaw=0.0)
        t_base_back_approx = make_tf(-0.4, 0.0, 0.7, yaw=np.pi) 
        t_base_bridge_approx = make_tf(0.32, 0.0, 1.32, yaw=np.pi) 

        # 2. Bridge基準での初期ヒントを計算
        t_bridge_base = np.linalg.inv(t_base_bridge_approx)
        hint_front = t_bridge_base @ t_base_front_true
        hint_back  = t_bridge_base @ t_base_back_approx

        # --- 可視化用の準備 ---
        bridge_temp = copy.deepcopy(pcds['bridge']).paint_uniform_color([0.5, 0.5, 0.5]) # グレー
        front_init = copy.deepcopy(pcds['front']).transform(hint_front).paint_uniform_color([1, 0, 0]) # 赤
        back_init  = copy.deepcopy(pcds['back']).transform(hint_back).paint_uniform_color([0, 0, 1])  # 青

        print("【確認1】初期状態（Front:赤, Back:青, Bridge:灰色）")
        print("地面が水平に重なっているか、壁の向きが合っているか確認してください。")
        o3d.visualization.draw_geometries([bridge_temp, front_init, back_init], window_name="1. Initial State")

        # 3. ICP実行
        def quick_icp(source, target, hint):
            reg = o3d.pipelines.registration.registration_icp(
                source, target, 0.1, hint,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            return reg.transformation

        t_bridge_front = quick_icp(pcds['front'], pcds['bridge'], hint_front)
        t_bridge_back  = quick_icp(pcds['back'], pcds['bridge'], hint_back)

        # --- 計算後の表示 ---
        front_final = copy.deepcopy(pcds['front']).transform(t_bridge_front).paint_uniform_color([0, 1, 0]) # 緑
        back_final  = copy.deepcopy(pcds['back']).transform(t_bridge_back).paint_uniform_color([0, 1, 1])  # 水色

        print("【確認2】ICP後（Front:緑, Back:水色, Bridge:灰色）")
        o3d.visualization.draw_geometries([bridge_temp, front_final, back_final], window_name="2. ICP Result")

        print("\n--- ICP Results ---")
        print(f"MAT_BRIDGE_FRONT = \n{repr(t_bridge_front)}")
        print(f"MAT_BRIDGE_BACK = \n{repr(t_bridge_back)}")
        rclpy.shutdown()

def main():
    rclpy.init(); rclpy.spin(BridgeMultiICP())
if __name__ == '__main__': main()