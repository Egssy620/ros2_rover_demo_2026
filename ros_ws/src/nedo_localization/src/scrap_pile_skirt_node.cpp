#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Scrap pile skirt node (ランドマークが見えている所で、自己位置と一緒に使う):
//   2026-10-16 本人指示: 実際は LM が見えている状況で使うので、単体 (LM・自己位置なし) で動かす作りをやめた。
//     - ランドマーク地図 (LM_ids / LM_xs / LM_ys、localizer と同じファイル) が必須。無ければ起動時に止まる
//     - map → ground_frame の TF (自己位置) が無いフレームは処理しない
//     - 出す点 (家電山・麓・物体の点・最近傍点) は map 系。表示用の markers (機体の箱・限界線など) だけ機体座標
//   1. 入力点群を ground_frame (base_footprint, Z=0 が地面) に変換
//   2. 物体候補 = 地面から skirt_z_min..wall_z_min の点
//      - 機体矩形 (+margin) の内側は自機の映り込みとして除外
//      - 水平距離 max_range より遠い点は除外
//      - ランドマークの円柱 (地図の中心から landmark_radius + landmark_margin 以内) は家電と間違えないよう除外
//      - 壁除去: 真上の列 (XYセル+周囲1セル) に wall_z_min..wall_z_max の点がある点は壁とみなして除外
//        (家電/山は背が低く、壁は天井近くまで点が続く。上限は天井の点を拾わないため)
//   3. 物体候補を3Dクラスタリングして家電山のクラスタを1つ選ぶ (cluster_select)
//   4. 麓の抽出: クラスタのうち skirt_z_max 以下の点について、センサ直下を中心とした床面上の
//      方位ビンごとにセンサに最も近い1点だけ残す (床を這ってきた視線が最初に山に当たる点 = 山が床から立ち上がる線)
//   5. 家電山クラスタ (薄い赤)、麓 (赤)、機体矩形から最も近い物体の点 (白) を publish
//   6. 把持・接近の判定 (アーム/グラップル未搭載のため全て変更可能なパラメータ):
//      - 把持可能範囲: アームは旋回しない (向き arm_yaw_deg で機体に固定)。グラップル中心はアームに沿って
//        根元から r_min..r を動き、開いたグラップルはその周り半径 d/2 を覆う
//        → 範囲 = 線分 (r_min..r) から d/2 以内の帯 (両端は半円)。表示のみ (中の点は色分けしない)
//      - 接近限界: 最近傍距離 < D なら近づきすぎ (~/too_close=true、距離表示と限界線を赤)
//        最近傍は裾だけでなく、物体の点 (~/object_points、全クラスタ・山の上の方を含む) から取る。
//        移動中の衝突防止に simple_goal_follower_node が使う (2026-09-29 移動中に山へ衝突したため)
//
//   rviz2 表示:
//     ~/cluster_points : PointCloud2 家電山クラスタ (麓の点を除く、rgb=薄い赤)
//     ~/skirt_points   : PointCloud2 麓 (rgb=赤)
//     ~/markers        : MarkerArray (白球=最近傍点, 白線=機体→最近傍, 白の距離テキスト, 機体の箱)
//     ~/nearest_point : PointStamped 機体矩形に最も近い点 (後段ノード用、map 系)
//     ~/pile_points    : PointCloud2 家電山クラスタ全点 (麓を含む)
//     ~/object_points  : PointCloud2 min_cluster_size 以上の全クラスタの点 (壁除去後、map 系)。pickup_planner_node 用
//                        (選んだ1クラスタだけだと、ピックアップで山が割れた時に残りが見えなくなる)
//     ~/too_close      : Bool 最近傍距離 < D
//     ~/markers には 接近限界線 (機体矩形+D, 黄/赤)・把持可能範囲の帯 (緑) も含む
//
//   動かす/止める (2026-10-06 本人指示: 山の前に着いたら起動、満杯などで山のループを抜けたら停止):
//     ~/enable  : SetBool。false の間は点群を処理せず何も出さない。止めたとき RViz の表示を消す
//                 (点群は空のものを1回出す。RViz は最後に届いた点群を出し続けるため)。マスターが呼ぶ
//     ~/enabled : Bool (transient_local) 今 動いているか。simple_goal_follower_node は false の間 too_close を待たない
//     start_enabled (パラメータ): 起動時に動かすか。既定 false (マスターが山の前で動かす)

class ScrapPileSkirtNode : public rclcpp::Node
{
public:
    ScrapPileSkirtNode()
    : Node("scrap_pile_skirt_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/lidar_points");
        ground_frame_ = this->declare_parameter<std::string>("ground_frame", "base_footprint");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        // ランドマーク地図 (localizer と同じファイルを launch から渡す)
        lm_xs_ = this->declare_parameter<std::vector<double>>("LM_xs", std::vector<double>{});
        lm_ys_ = this->declare_parameter<std::vector<double>>("LM_ys", std::vector<double>{});
        if (lm_xs_.empty() || lm_xs_.size() != lm_ys_.size())
            throw std::runtime_error("ランドマーク地図 (LM_xs / LM_ys) がない。landmark_map_file を渡して起動する");
        // 除外半径 = 円柱の半径 (外径 413 mm、検出器の landmark_radius と同じ) + 余裕。
        //   余裕 = 自己位置のずれ (実測 1〜5 cm) と QT128 の距離のばらつき (数 cm) の分。仮値
        landmark_radius_ = this->declare_parameter<double>("landmark_radius", 0.20675);
        landmark_margin_ = this->declare_parameter<double>("landmark_margin", 0.10);
        transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.1);

        // 裾の高さ帯 [m, 地面基準]。下限は地面点のばらつきを切る値、上限が「裾」の定義
        skirt_z_min_ = this->declare_parameter<double>("skirt_z_min", 0.05);
        skirt_z_max_ = this->declare_parameter<double>("skirt_z_max", 0.30);
        max_range_ = this->declare_parameter<double>("max_range", 10.0);

        // 機体矩形 [m, ground_frame]。最近傍距離の基準、かつ内側(+margin)は自機として除外
        body_x_min_ = this->declare_parameter<double>("body_x_min", -0.5);
        body_x_max_ = this->declare_parameter<double>("body_x_max", 0.5);
        body_y_min_ = this->declare_parameter<double>("body_y_min", -0.4);
        body_y_max_ = this->declare_parameter<double>("body_y_max", 0.4);
        body_margin_ = this->declare_parameter<double>("body_margin", 0.10);
        // 機体の高さ [m, 表示用]。仮値: LiDAR (機体の左上) の床からの高さ 1.94 m
        body_height_ = this->declare_parameter<double>("body_height", 1.94);

        // クラスタ選択: "largest"(最大点数) / "nearest"(機体に最も近い) / "none"(帯の全点)
        //   既定 nearest: 2026-09-24 QT128 bag④で largest は常設の低い物体 (34点) を選び、家電 (16点) を取り逃した
        cluster_select_ = this->declare_parameter<std::string>("cluster_select", "nearest");
        if (cluster_select_ != "largest" && cluster_select_ != "nearest" && cluster_select_ != "none")
        {
            RCLCPP_WARN(get_logger(), "unknown cluster_select '%s' -> largest", cluster_select_.c_str());
            cluster_select_ = "largest";
        }
        // 壁除去。2026-09-24 QT128 bag 4本の実測: 対象(家電/山)の列の最大高さ p90=0.71m、壁は中央値2.5m・p10≥1.4m、天井2.98m
        wall_filter_ = this->declare_parameter<bool>("wall_filter", true);
        wall_cell_size_ = this->declare_parameter<double>("wall_cell_size", 0.10);
        wall_z_min_ = this->declare_parameter<double>("wall_z_min", 1.2);
        wall_z_max_ = this->declare_parameter<double>("wall_z_max", 2.7);

        // 麓の方位ビン幅 [deg]。1ビン1点なので、麓線の点間隔 ≈ 距離×この角度
        foot_only_ = this->declare_parameter<bool>("foot_only", true);
        foot_bin_deg_ = this->declare_parameter<double>("foot_bin_deg", 1.0);

        // アーム・グラップル [m, ground_frame]。実機未搭載のため全て仮値
        //   アームは機体の長辺のうち LiDAR 側 (左, +y) から、長辺に直角に伸びる (2026-09-25 ユーザー情報)
        //   arm_base_x/y: アーム根元の位置。y = 左の長辺 (body_y_max)、x は長辺上のどこか未定のため仮値=中央
        //   arm_yaw_deg: アームの向き (機体 +x から反時計回り)。アームは旋回しない。左の長辺に直角 = +90
        //   grapple_min_reach (r_min): グラップル中心が最も手前に来たときの根元からの水平距離
        //   arm_length (r): 根元からグラップル中心までの最大水平距離
        //   grapple_open_diameter (d): グラップルを開いたときの直径
        //   approach_limit (D): 機体矩形と家電の最小許容距離。最近傍距離がこれを下回ったら too_close
        arm_base_x_ = this->declare_parameter<double>("arm_base_x", 0.0);
        arm_base_y_ = this->declare_parameter<double>("arm_base_y", 0.4);
        arm_yaw_ = this->declare_parameter<double>("arm_yaw_deg", 90.0) * M_PI / 180.0;
        grapple_min_reach_ = this->declare_parameter<double>("grapple_min_reach", 0.0);
        arm_length_ = this->declare_parameter<double>("arm_length", 0.7);
        // grapple_fixed: グラップルをアーム先端 (r) に固定する (2026-09-25 まず簡単のため)。
        //   true なら r_min = r で、掴める範囲は帯でなく先端の円 (直径 d) 1つ
        if (this->declare_parameter<bool>("grapple_fixed", true)) grapple_min_reach_ = arm_length_;
        grapple_open_diameter_ = this->declare_parameter<double>("grapple_open_diameter", 0.6);
        approach_limit_ = this->declare_parameter<double>("approach_limit", 0.3);

        cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.30);
        // 2026-09-24 実測: 孤立点の塊は5〜8点、最小の家電(bag④)は16点
        min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 10);

        pub_skirt_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/skirt_points", 1);
        pub_cluster_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/cluster_points", 1);
        pub_nearest_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/nearest_point", 1);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);
        pub_pile_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pile_points", 1);
        pub_objects_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/object_points", 1);
        pub_too_close_ = this->create_publisher<std_msgs::msg::Bool>("~/too_close", 1);
        // 後から起動したノードにも今の状態が分かるように、最後の1つを残す
        pub_enabled_ = this->create_publisher<std_msgs::msg::Bool>("~/enabled", rclcpp::QoS(1).transient_local());

        enabled_ = this->declare_parameter<bool>("start_enabled", false);
        publish_enabled();
        srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
            "~/enable", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                if (enabled_ != req->data)
                {
                    enabled_ = req->data;
                    RCLCPP_INFO(get_logger(), "山検出を%s", enabled_ ? "開始" : "停止");
                    if (!enabled_) clear_display();
                    publish_enabled();
                }
                res->success = true;
                res->message = enabled_ ? "動作中" : "停止中";
            });

        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&ScrapPileSkirtNode::pointcloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "ScrapPileSkirtNode started (%s). input=%s frame=%s landmarks=%zu skirt_z=[%.2f, %.2f] select=%s "
                    "arm r_min=%.2f r=%.2f d=%.2f yaw=%.1fdeg base=(%.2f, %.2f) approach D=%.2f",
                    enabled_ ? "動作中" : "停止中 (~/enable で開始)",
                    input_topic_.c_str(), ground_frame_.c_str(), lm_xs_.size(), skirt_z_min_, skirt_z_max_,
                    cluster_select_.c_str(), grapple_min_reach_, arm_length_, grapple_open_diameter_,
                    arm_yaw_ * 180.0 / M_PI, arm_base_x_, arm_base_y_, approach_limit_);
    }

private:
    void publish_enabled()
    {
        std_msgs::msg::Bool m;
        m.data = enabled_;
        pub_enabled_->publish(m);
    }

    // 止めたとき、最後の表示 (点群・最近傍点・距離・限界線など) が残らないように消す
    void clear_display()
    {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(m);
        pub_markers_->publish(arr);
        const pcl::PointCloud<pcl::PointXYZ> none;
        for (const auto &pub : {pub_skirt_, pub_cluster_, pub_pile_, pub_objects_})
            publish_colored(pub, now(), none, {}, 0, 0, 0);
    }

    // 機体矩形から点 (x, y) までの水平距離。矩形の内側なら 0
    double distance_to_body(double x, double y, double margin) const
    {
        double dx = std::max({body_x_min_ - margin - x, 0.0, x - (body_x_max_ + margin)});
        double dy = std::max({body_y_min_ - margin - y, 0.0, y - (body_y_max_ + margin)});
        return std::hypot(dx, dy);
    }

    int64_t cell_key(double x, double y) const
    {
        const int64_t ix = static_cast<int64_t>(std::floor(x / wall_cell_size_));
        const int64_t iy = static_cast<int64_t>(std::floor(y / wall_cell_size_));
        return (ix << 32) ^ (iy & 0xffffffffLL);
    }

    bool in_wall_cell(const std::unordered_set<int64_t> &wall_cells, double x, double y) const
    {
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                if (wall_cells.count(cell_key(x + dx * wall_cell_size_, y + dy * wall_cell_size_))) return true;
        return false;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!enabled_) return;

        // Step 1: ground_frame へ変換
        sensor_msgs::msg::PointCloud2 cloud_ground;
        try
        {
            auto tf = tf_buffer_.lookupTransform(
                ground_frame_, msg->header.frame_id, rclcpp::Time(0),
                rclcpp::Duration::from_seconds(transform_timeout_));
            tf2::doTransform(*msg, cloud_ground, tf);
            sensor_x_ = tf.transform.translation.x;
            sensor_y_ = tf.transform.translation.y;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "TF lookup failed (%s -> %s): %s",
                msg->header.frame_id.c_str(), ground_frame_.c_str(), ex.what());
            return;
        }

        // Step 1b: 自己位置 (map → ground_frame)。無ければ処理しない (ランドマークの位置も map 系で出す点も決まらない)
        try
        {
            const auto t = tf_buffer_.lookupTransform(map_frame_, ground_frame_, rclcpp::Time(0),
                                                      rclcpp::Duration::from_seconds(transform_timeout_)).transform;
            to_map_ = tf2::Transform(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                                     tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "自己位置がまだありません (%s -> %s)。山検出を待ちます: %s",
                ground_frame_.c_str(), map_frame_.c_str(), ex.what());
            return;
        }
        // ランドマークの中心を ground_frame へ
        std::vector<tf2::Vector3> landmarks;
        const tf2::Transform from_map = to_map_.inverse();
        for (size_t i = 0; i < lm_xs_.size(); ++i) landmarks.push_back(from_map * tf2::Vector3(lm_xs_[i], lm_ys_[i], 0.0));
        const double lm_r = landmark_radius_ + landmark_margin_;
        size_t n_landmark = 0;

        // Step 2: 物体候補 (距離・自機除外)。同じ走査で壁セル (高い点がある列) を集める
        pcl::PointCloud<pcl::PointXYZ> cand_all;
        std::unordered_set<int64_t> wall_cells;
        const double max_r2 = max_range_ * max_range_;
        for (sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_ground, "x"),
             it_y(cloud_ground, "y"), it_z(cloud_ground, "z");
             it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            const float x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (static_cast<double>(x) * x + static_cast<double>(y) * y > max_r2) continue;
            if (distance_to_body(x, y, body_margin_) == 0.0) continue;
            if (wall_filter_ && z >= wall_z_min_ && z <= wall_z_max_)
            {
                wall_cells.insert(cell_key(x, y));
            }
            if (z < skirt_z_min_ || z >= wall_z_min_) continue;
            if (std::any_of(landmarks.begin(), landmarks.end(), [&](const tf2::Vector3 &c) {
                    return std::hypot(x - c.x(), y - c.y()) < lm_r; }))
            {
                ++n_landmark;
                continue;
            }
            cand_all.points.emplace_back(x, y, z);
        }

        // Step 2b: 壁除去
        pcl::PointCloud<pcl::PointXYZ>::Ptr cand(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &p : cand_all.points)
        {
            if (wall_filter_ && in_wall_cell(wall_cells, p.x, p.y)) continue;
            cand->points.push_back(p);
        }
        cand->width = cand->points.size();
        cand->height = 1;

        // Step 3: 家電山クラスタを選ぶ
        std::vector<int> cluster, objects;
        if (cluster_select_ == "none")
        {
            cluster.resize(cand->points.size());
            for (size_t i = 0; i < cluster.size(); ++i) cluster[i] = static_cast<int>(i);
        }
        else if (!cand->points.empty())
        {
            cluster = select_cluster(cand, objects);
        }

        // Step 4: 麓の抽出 (クラスタの低い部分から)
        std::vector<int> low;
        for (int idx : cluster)
        {
            if (cand->points[idx].z <= skirt_z_max_) low.push_back(idx);
        }
        const std::vector<int> skirt = foot_only_ ? extract_foot(*cand, low) : low;

        // Step 5: 最近傍点 (機体矩形からの水平距離が最小) と接近限界。
        //   対象 = 物体の点 (~/object_points と同じ: 高さ skirt_z_min..wall_z_min、壁除去後、min_cluster_size 以上の全クラスタ)。
        //   裾だけでなく山の上の方や別の物体も数える。wall_z_min より上は数えない
        //   (2026-09-29: 機体の高さまでの全点にしたら、センサ付近 1.6〜2.0 m の構造物を 0.10 m に拾い続けた)
        pcl::PointXYZ nearest;
        double nearest_d = std::numeric_limits<double>::infinity();
        for (int idx : cluster_select_ == "none" ? cluster : objects)
        {
            const auto &p = cand->points[idx];
            const double d = distance_to_body(p.x, p.y, 0.0);
            if (d < nearest_d)
            {
                nearest_d = d;
                nearest = p;
            }
        }
        const bool has_nearest = std::isfinite(nearest_d);
        const bool too_close = has_nearest && nearest_d < approach_limit_;

        // クラスタ表示からは麓の点を除く (赤が薄い赤に隠れないように)
        std::unordered_set<int> skirt_set(skirt.begin(), skirt.end());
        std::vector<int> cluster_rest;
        for (int idx : cluster)
        {
            if (!skirt_set.count(idx)) cluster_rest.push_back(idx);
        }

        publish_colored(pub_skirt_, msg->header.stamp, *cand, skirt, 255, 0, 0);              // 赤
        publish_colored(pub_cluster_, msg->header.stamp, *cand, cluster_rest, 255, 170, 170); // 薄い赤
        publish_colored(pub_pile_, msg->header.stamp, *cand, cluster, 255, 170, 170);
        publish_colored(pub_objects_, msg->header.stamp, *cand, cluster_select_ == "none" ? cluster : objects,
                        200, 200, 200);
        publish_markers(msg->header.stamp, has_nearest ? &nearest : nullptr, nearest_d, too_close);

        std_msgs::msg::Bool tc;
        tc.data = too_close;
        pub_too_close_->publish(tc);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "cand=%zu landmark=%zu wall=%zu cluster=%zu skirt=%zu nearest=%.2fm%s",
            cand_all.points.size(), n_landmark, cand_all.points.size() - cand->points.size(),
            cluster.size(), skirt.size(), nearest_d,
            too_close ? " TOO CLOSE (< approach_limit)" : "");
    }

    std::vector<int> extract_foot(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                  const std::vector<int> &indices) const
    {
        // key: 方位ビン番号, value: (センサからの水平距離, 点index)
        std::unordered_map<int, std::pair<double, int>> best;
        const double bin_rad = foot_bin_deg_ * M_PI / 180.0;
        for (int idx : indices)
        {
            const auto &p = cloud.points[idx];
            const double dx = p.x - sensor_x_, dy = p.y - sensor_y_;
            const int bin = static_cast<int>(std::floor(std::atan2(dy, dx) / bin_rad));
            const double r = std::hypot(dx, dy);
            auto it = best.find(bin);
            if (it == best.end() || r < it->second.first) best[bin] = {r, idx};
        }
        std::vector<int> foot;
        foot.reserve(best.size());
        for (const auto &kv : best) foot.push_back(kv.second.second);
        return foot;
    }

    // 選んだクラスタを返し、objects に min_cluster_size 以上の全クラスタの点を入れる
    std::vector<int> select_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &objects) const
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(static_cast<int>(cloud->points.size()));
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusters);
        for (const auto &c : clusters) objects.insert(objects.end(), c.indices.begin(), c.indices.end());
        if (clusters.empty()) return {};

        size_t best = 0;
        double best_score = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            double score;
            if (cluster_select_ == "largest")
            {
                score = -static_cast<double>(clusters[i].indices.size());
            }
            else
            {
                score = std::numeric_limits<double>::infinity();
                for (int idx : clusters[i].indices)
                {
                    const auto &p = cloud->points[idx];
                    score = std::min(score, distance_to_body(p.x, p.y, 0.0));
                }
            }
            if (score < best_score)
            {
                best_score = score;
                best = i;
            }
        }
        return clusters[best].indices;
    }

    void publish_colored(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
                         const builtin_interfaces::msg::Time &stamp,
                         const pcl::PointCloud<pcl::PointXYZ> &cloud,
                         const std::vector<int> &indices, uint8_t r, uint8_t g, uint8_t b)
    {
        sensor_msgs::msg::PointCloud2 out;
        out.header.stamp = stamp;
        out.header.frame_id = map_frame_;  // ground_frame で計算した点を map 系にして出す
        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
        mod.resize(indices.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(out, "x"), it_y(out, "y"), it_z(out, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> it_r(out, "r"), it_g(out, "g"), it_b(out, "b");
        for (int idx : indices)
        {
            const tf2::Vector3 p = to_map_ * tf2::Vector3(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);
            *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
            *it_r = r; *it_g = g; *it_b = b;
            ++it_x; ++it_y; ++it_z; ++it_r; ++it_g; ++it_b;
        }
        pub->publish(out);
    }

    void publish_markers(const builtin_interfaces::msg::Time &stamp,
                         const pcl::PointXYZ *nearest, double nearest_d, bool too_close)
    {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker base;
        base.header.stamp = stamp;
        base.header.frame_id = ground_frame_;
        base.ns = "scrap_pile_skirt";
        base.pose.orientation.w = 1.0;

        // 機体の箱 (灰色の枠線、床 z=0 から body_height まで)
        auto body = base;
        body.id = 0;
        body.type = visualization_msgs::msg::Marker::LINE_LIST;
        body.scale.x = 0.02;
        body.color.r = body.color.g = body.color.b = 0.6f;
        body.color.a = 1.0f;
        auto corner = [&](int i) {
            // i: bit0=x, bit1=y, bit2=z
            geometry_msgs::msg::Point p;
            p.x = (i & 1) ? body_x_max_ : body_x_min_;
            p.y = (i & 2) ? body_y_max_ : body_y_min_;
            p.z = (i & 4) ? body_height_ : 0.0;
            return p;
        };
        for (int i = 0; i < 8; ++i)
        {
            for (int bit = 1; bit <= 4; bit <<= 1)
            {
                if (i & bit) continue;  // 各辺を1回ずつ
                body.points.push_back(corner(i));
                body.points.push_back(corner(i | bit));
            }
        }
        arr.markers.push_back(body);

        // 接近限界線: 機体矩形を D だけ外側に広げた角丸矩形 (床上)。通常は黄、近づきすぎで赤
        auto limit = base;
        limit.id = 4;
        limit.type = visualization_msgs::msg::Marker::LINE_STRIP;
        limit.scale.x = 0.02;
        limit.color.r = 1.0f;
        limit.color.g = too_close ? 0.0f : 0.85f;
        limit.color.a = 1.0f;
        {
            const double D = approach_limit_;
            // 角の中心 (反時計回り) と、各角で描く円弧の開始角
            const double cx[4] = {body_x_max_, body_x_min_, body_x_min_, body_x_max_};
            const double cy[4] = {body_y_max_, body_y_max_, body_y_min_, body_y_min_};
            const int arc_steps = 8;
            for (int c = 0; c < 4; ++c)
            {
                for (int k = 0; k <= arc_steps; ++k)
                {
                    const double th = (c + static_cast<double>(k) / arc_steps) * M_PI / 2.0;
                    geometry_msgs::msg::Point p;
                    p.x = cx[c] + D * std::cos(th);
                    p.y = cy[c] + D * std::sin(th);
                    p.z = 0.01;
                    limit.points.push_back(p);
                }
            }
            limit.points.push_back(limit.points.front());
        }
        arr.markers.push_back(limit);

        // 把持可能範囲の帯 (床上): グラップル中心 (緑の点) と、そこから d/2 以内の輪郭 (緑、両端は半円)。
        // グラップル固定 (r_min = r) なら直径 d の円と中心点
        {
            const double c = std::cos(arm_yaw_), sn = std::sin(arm_yaw_);
            auto to_ground = [&](double along, double side) {
                geometry_msgs::msg::Point p;
                p.x = arm_base_x_ + along * c - side * sn;
                p.y = arm_base_y_ + along * sn + side * c;
                p.z = body_height_;  // 点群より上に描く (上面図で点に隠れないように)
                return p;
            };
            // グラップル中心 (緑の点)。r_min < r なら動ける線分の両端
            auto path = base;
            path.id = 5;
            path.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            path.scale.x = path.scale.y = path.scale.z = 0.08;
            path.color.g = 0.86f;
            path.color.a = 1.0f;
            path.points = {to_ground(grapple_min_reach_, 0.0), to_ground(arm_length_, 0.0)};
            arr.markers.push_back(path);

            auto zone = base;
            zone.id = 6;
            zone.type = visualization_msgs::msg::Marker::LINE_STRIP;
            zone.scale.x = 0.02;
            zone.color.g = 0.86f;
            zone.color.a = 1.0f;
            const double h = 0.5 * grapple_open_diameter_;
            const int arc_steps = 24;
            for (int k = 0; k <= arc_steps; ++k)  // 奥の半円 (右 → 左)
            {
                const double th = -M_PI / 2.0 + M_PI * k / arc_steps;
                zone.points.push_back(to_ground(arm_length_ + h * std::cos(th), h * std::sin(th)));
            }
            for (int k = 0; k <= arc_steps; ++k)  // 手前の半円 (左 → 右)
            {
                const double th = M_PI / 2.0 + M_PI * k / arc_steps;
                zone.points.push_back(to_ground(grapple_min_reach_ + h * std::cos(th), h * std::sin(th)));
            }
            zone.points.push_back(zone.points.front());
            arr.markers.push_back(zone);
        }

        auto sphere = base;
        sphere.id = 1;
        auto line = base;
        line.id = 2;
        auto text = base;
        text.id = 3;

        if (nearest == nullptr)
        {
            sphere.action = line.action = text.action = visualization_msgs::msg::Marker::DELETE;
            arr.markers.push_back(sphere);
            arr.markers.push_back(line);
            arr.markers.push_back(text);
            pub_markers_->publish(arr);
            return;
        }

        // 最近傍点 (白球)
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.pose.position.x = nearest->x;
        sphere.pose.position.y = nearest->y;
        sphere.pose.position.z = nearest->z;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
        sphere.color.r = sphere.color.g = sphere.color.b = 1.0f;
        sphere.color.a = 1.0f;
        arr.markers.push_back(sphere);

        // 機体矩形上の最近点 → 最近傍点 (白線)
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.scale.x = 0.03;
        line.color.r = line.color.g = line.color.b = 1.0f;
        line.color.a = 1.0f;
        geometry_msgs::msg::Point a, b;
        a.x = std::clamp(static_cast<double>(nearest->x), body_x_min_, body_x_max_);
        a.y = std::clamp(static_cast<double>(nearest->y), body_y_min_, body_y_max_);
        a.z = nearest->z;
        b.x = nearest->x; b.y = nearest->y; b.z = nearest->z;
        line.points = {a, b};
        arr.markers.push_back(line);

        // 距離テキスト (白)。点や線と重ならないよう、線の中点から線に直交する向きへ 0.3 m ずらす
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        const double lx = b.x - a.x, ly = b.y - a.y;
        const double ll = std::hypot(lx, ly);
        const double nx = ll > 1e-6 ? -ly / ll : 0.0, ny = ll > 1e-6 ? lx / ll : 1.0;
        text.pose.position.x = 0.5 * (a.x + b.x) + 0.3 * nx;
        text.pose.position.y = 0.5 * (a.y + b.y) + 0.3 * ny;
        text.pose.position.z = nearest->z + 0.3;
        text.scale.z = 0.2;
        // 近づきすぎ (< D) なら赤
        text.color.r = 1.0f;
        text.color.g = text.color.b = too_close ? 0.0f : 1.0f;
        text.color.a = 1.0f;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2fm", nearest_d);
        text.text = buf;
        arr.markers.push_back(text);

        pub_markers_->publish(arr);

        geometry_msgs::msg::PointStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = map_frame_;
        const tf2::Vector3 bm = to_map_ * tf2::Vector3(b.x, b.y, b.z);
        ps.point.x = bm.x();
        ps.point.y = bm.y();
        ps.point.z = bm.z();
        pub_nearest_->publish(ps);
    }

    std::string input_topic_, ground_frame_, map_frame_;
    std::vector<double> lm_xs_, lm_ys_;
    double landmark_radius_{}, landmark_margin_{};
    tf2::Transform to_map_{tf2::Transform::getIdentity()};  // ground_frame → map (点群ごとに更新)
    double transform_timeout_{};
    double skirt_z_min_{}, skirt_z_max_{}, max_range_{};
    double body_x_min_{}, body_x_max_{}, body_y_min_{}, body_y_max_{}, body_margin_{}, body_height_{};
    bool foot_only_{};
    double foot_bin_deg_{};
    double sensor_x_{}, sensor_y_{};
    double arm_base_x_{}, arm_base_y_{}, arm_yaw_{}, grapple_min_reach_{}, arm_length_{};
    double grapple_open_diameter_{}, approach_limit_{};
    bool wall_filter_{};
    double wall_cell_size_{}, wall_z_min_{}, wall_z_max_{};
    std::string cluster_select_;
    double cluster_tolerance_{};
    int min_cluster_size_{};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_skirt_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_nearest_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pile_, pub_objects_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_too_close_, pub_enabled_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
    bool enabled_{true};
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScrapPileSkirtNode>());
    rclcpp::shutdown();
    return 0;
}
