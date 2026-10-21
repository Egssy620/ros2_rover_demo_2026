#include "nedo_localization/localizer.hpp"

namespace nedo_localization
{
    Localizer::Localizer(const rclcpp::NodeOptions &options)
        : Node("localizer_node", options), tf_broadcaster_(this)
    {
        marker_suber_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
            "matched_marker",
            rclcpp::SensorDataQoS(),
            std::bind(&Localizer::marker_pose_callback, this, std::placeholders::_1));
        pose_puber_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", rclcpp::SystemDefaultsQoS());
        load_map();
        subscribe_map_topic();
        calculate_success_ = false;
        // 旧実装は true 初期化だったが、初回 publish 前に previous_pose_ は未設定なので
        // lpf_pose が (0,0,0) と混ぜてしまう。初回 publish が完了してから true にする。
        has_previous_pose_ = false;
        lpf_rate_ = this->declare_parameter<double>("lpf_rate", 0.0);
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        // map->base_link の動的TFを配信するか (default true)。
        // EKF 併用時は false にして EKF に TF を所有させる。
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    }

    void Localizer::marker_pose_callback(const vehicle_interface::msg::PoseStampedArray::SharedPtr marker_poses)
    {
        // 現フレームのみを SVD に使う方針 (過去フレームを混ぜると base_link 系の移動で
        // 系統的バイアスが発生する)。callback 先頭で current_detections_ を clear。
        current_detections_.clear();
        for (const auto &marker_pose : marker_poses->array)
        {
            current_detections_[marker_pose.header.frame_id] = marker_pose;
        }
        localization();
    }

    void Localizer::localization()
    {
        calculate_success_ = true;
        geometry_msgs::msg::PoseStamped pub_msg;

        localize_by_svd(pub_msg);
        if (!calculate_success_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "failed to calculate Pose (detections < 2 or SVD失敗)");
            return;
        }

        pub_msg.header.frame_id = map_frame_;
        pub_msg.header.stamp = this->now();

        // 平滑化 (lpf_rate=0 なら何もしない / 初回は has_previous_pose_=false なのでスキップ)
        lpf_pose(pub_msg);

        pose_puber_->publish(pub_msg);
        previous_pose_ = pub_msg;
        has_previous_pose_ = true;   // 初回 publish 完了後に true へ
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "publish Pose");

        // map -> base_link の動的TF配信 (SVD 解そのまま使用)
        // 旧実装は try/catch していたが sendTransform は例外を投げないため削除した。
        // publish_tf_=false (EKF併用時) は配信しない。EKF が odom->base_link を所有し、
        // map->base_link は static map->odom と合成で得るため二重親を避ける。
        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = pub_msg.header.stamp;
            tf.header.frame_id = map_frame_;
            tf.child_frame_id = "base_link";
            tf.transform.translation.x = pub_msg.pose.position.x;
            tf.transform.translation.y = pub_msg.pose.position.y;
            tf.transform.translation.z = pub_msg.pose.position.z;
            tf.transform.rotation = pub_msg.pose.orientation;
            tf_broadcaster_.sendTransform(tf);
        }
    }

    void Localizer::load_map()
    {
        // まず新しい形式（landmark_ids + marker{N}.position）を試行
        std::vector<std::string> landmark_ids;
        bool use_new_format = false;

        try
        {
            landmark_ids = this->declare_parameter<std::vector<std::string>>("landmark_ids", std::vector<std::string>());
            if (!landmark_ids.empty())
            {
                use_new_format = true;
                RCLCPP_INFO(this->get_logger(), "Using new landmark parameter format (landmark_ids + marker{N}.position)");
            }
        }
        catch (...)
        {
            // パラメータが存在しない場合は旧形式を使用
        }

        if (use_new_format)
        {
            // 新しい形式：landmark_ids + marker{N}.position
            for (const auto &id : landmark_ids)
            {
                try
                {
                    std::vector<double> position = this->declare_parameter<std::vector<double>>(id + ".position", std::vector<double>());
                    if (position.size() >= 2)
                    {
                        geometry_msgs::msg::PoseStamped landmark;
                        landmark.header.stamp = this->now();
                        landmark.header.frame_id = map_frame_;
                        landmark.pose.position.x = position[0];
                        landmark.pose.position.y = position[1];
                        landmark.pose.position.z = (position.size() >= 3) ? position[2] : 0.0;
                        landmark.pose.orientation.w = 1.0; // デフォルトの向き

                        world_map_[id] = landmark;
                        RCLCPP_INFO(this->get_logger(), "Loaded landmark - id:%s, x:%f, y:%f, z:%f",
                                    id.c_str(), position[0], position[1],
                                    (position.size() >= 3) ? position[2] : 0.0);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Invalid position for landmark %s (need at least x,y)", id.c_str());
                    }
                }
                catch (...)
                {
                    RCLCPP_WARN(this->get_logger(), "Could not load position for landmark %s", id.c_str());
                }
            }
        }
        else
        {
            // 旧形式：LM_ids, LM_xs, LM_ys
            RCLCPP_INFO(this->get_logger(), "Using legacy landmark parameter format (LM_ids, LM_xs, LM_ys)");
            std::vector<std::string> ids = this->declare_parameter<std::vector<std::string>>("LM_ids", std::vector<std::string>());
            std::vector<double> xs = this->declare_parameter<std::vector<double>>("LM_xs", std::vector<double>());
            std::vector<double> ys = this->declare_parameter<std::vector<double>>("LM_ys", std::vector<double>());

            if (xs.size() != ids.size() || ys.size() != ids.size())
            {
                RCLCPP_ERROR(this->get_logger(), "Legacy parameters are not defined correctly!!");
                return;
            }

            for (size_t i = 0; i < ids.size(); i++)
            {
                geometry_msgs::msg::PoseStamped landmark;
                landmark.header.stamp = this->now();
                landmark.header.frame_id = map_frame_;
                landmark.pose.position.x = xs.at(i);
                landmark.pose.position.y = ys.at(i);
                landmark.pose.position.z = 0.0;
                landmark.pose.orientation.w = 1.0;

                world_map_[ids.at(i)] = landmark;
                RCLCPP_INFO(this->get_logger(), "Loaded legacy landmark - id:%s, x:%f, y:%f",
                            ids[i].c_str(), xs[i], ys[i]);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu landmarks into world map", world_map_.size());
    }

    void Localizer::subscribe_map_topic()
    {
        // Subscribe to landmark map topic for dynamic updates
        map_suber_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
            "/landmark_map", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&Localizer::handle_map_msg, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmark_map for dynamic map updates");
    }

    void Localizer::handle_map_msg(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
    {
        // Update world_map_ from topic
        std::map<std::string, geometry_msgs::msg::PoseStamped> new_map;
        for (const auto &p : msg->array)
        {
            // id is carried in header.frame_id
            if (!p.header.frame_id.empty())
            {
                geometry_msgs::msg::PoseStamped m = p;
                m.header.frame_id = map_frame_; // ensure global frame
                new_map[p.header.frame_id] = m;
            }
        }
        if (!new_map.empty())
        {
            world_map_.swap(new_map);
            RCLCPP_INFO(this->get_logger(), "World map updated from topic with %zu landmarks", world_map_.size());
        }
    }

    // localization using svd method
    //
    // 現フレームで検出された ID のうち world_map に存在するもののペアを集めて
    // 2D 剛体変換 (R, t) を SVD で求める。必要な対応ペアは 2 個以上 (2D pose
    // 3自由度を方程式 4本以上で over-determined に解く)。
    void Localizer::localize_by_svd(geometry_msgs::msg::PoseStamped &robot_pose)
    {
        std::vector<Eigen::Vector2d> detectVec;
        std::vector<Eigen::Vector2d> worldVec;

        for (const auto &kv : current_detections_)
        {
            const std::string &id = kv.first;
            const auto wm_it = world_map_.find(id);
            if (wm_it == world_map_.end()) continue;
            detectVec.emplace_back(Eigen::Vector2d(kv.second.pose.position.x, kv.second.pose.position.y));
            worldVec.emplace_back(Eigen::Vector2d(wm_it->second.pose.position.x, wm_it->second.pose.position.y));
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Valid landmarks count: %zu (current frame: %zu)",
                             detectVec.size(), current_detections_.size());

        if (detectVec.size() < 2)
        {
            calculate_success_ = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Insufficient landmarks for localization: %zu (need >=2)", detectVec.size());
            return;
        }
        best_fit_transform(detectVec, worldVec, robot_pose);
    }

    void Localizer::lpf_pose(geometry_msgs::msg::PoseStamped &current_pose)
    {
        if (has_previous_pose_)
        {
            current_pose.pose.position.x = previous_pose_.pose.position.x * lpf_rate_ + current_pose.pose.position.x * (1 - lpf_rate_);
            current_pose.pose.position.y = previous_pose_.pose.position.y * lpf_rate_ + current_pose.pose.position.y * (1 - lpf_rate_);
            current_pose.pose.position.z = previous_pose_.pose.position.z * lpf_rate_ + current_pose.pose.position.z * (1 - lpf_rate_);
        }
    }

    // Function to calculate the best fit transform in 2D
    void Localizer::best_fit_transform(
        const std::vector<Eigen::Vector2d> &A, const std::vector<Eigen::Vector2d> &B, geometry_msgs::msg::PoseStamped &return_transform)
    {

        if (A.size() != B.size() || A.size() < 2)
        {
            throw std::invalid_argument("Point sets must have the same size and contain at least 2 points.");
        }

        // Step 1: Compute centroids of A and B
        Eigen::Vector2d centroid_A = Eigen::Vector2d::Zero();
        Eigen::Vector2d centroid_B = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < A.size(); ++i)
        {
            centroid_A += A[i];
            centroid_B += B[i];
        }
        centroid_A /= A.size();
        centroid_B /= B.size();

        // Step 2: Subtract centroids to shift points to the origin
        std::vector<Eigen::Vector2d> AA(A.size());
        std::vector<Eigen::Vector2d> BB(B.size());
        for (size_t i = 0; i < A.size(); ++i)
        {
            AA[i] = A[i] - centroid_A;
            BB[i] = B[i] - centroid_B;
        }

        // Step 3: Compute the covariance matrix H
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < A.size(); ++i)
        {
            H += AA[i] * BB[i].transpose();
        }

        // Step 4: Perform SVD on the covariance matrix H
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();

        // Step 5: Compute the rotation matrix R
        Eigen::Matrix2d R = V * U.transpose();

        // Handle the case where the determinant of R is negative
        if (R.determinant() < 0)
        {
            V.col(1) *= -1; // Flip the second column of V
            R = V * U.transpose();
        }

        // Step 6: Compute the translation vector t
        Eigen::Vector2d t = centroid_B - R * centroid_A;

        // Return the rotation matrix and translation vector
        return_transform.pose.position.x = t(0);
        return_transform.pose.position.y = t(1);
        auto theta = std::atan2(R(1, 0), R(0, 0));
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        tf2::convert(q, return_transform.pose.orientation);
    }
} // nedo_localization

// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::Localizer)