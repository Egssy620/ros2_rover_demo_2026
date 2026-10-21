#include"nedo_localization/projection_util.hpp"

namespace nedo_localization{
    std::vector<Eigen::RowVectorXd> project_image_coordinate(
        const sensor_msgs::msg::CameraInfo &caminfo,
        const geometry_msgs::msg::TransformStamped &cam_to_lidar,
        const pcl::PointCloud<pcl::PointXYZ> &pointcloud,
        bool opencv_ver
    ) {
        std::cerr << "[project_image_coordinate] CALLED with " << pointcloud.points.size() 
                  << " points, caminfo width=" << caminfo.width << std::endl;
        // カメラ行列の作成
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
            caminfo.k[0], caminfo.k[1], caminfo.k[2],  // fx, 0, cx
            caminfo.k[3], caminfo.k[4], caminfo.k[5],  // 0, fy, cy
            caminfo.k[6], caminfo.k[7], caminfo.k[8]); // 0,  0,  1

        // 歪み係数
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 
            caminfo.d[0], caminfo.d[1], caminfo.d[2], caminfo.d[3], caminfo.d[4]);

        // 回転ベクトルと並進ベクトルを作成
        // 注意: objectPointsにはすでにカメラ座標系に変換済みの点を渡すため、
        // rvec/tvecは恒等変換（ゼロ）とする
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        
        // カメラ座標系への変換行列を準備（フィルタリングと座標変換用）
        // 注意: 引数名はcam_to_lidarだが、実際にはlidar→cameraの変換が渡されている
        Eigen::Quaterniond q(
            cam_to_lidar.transform.rotation.w,
            cam_to_lidar.transform.rotation.x,
            cam_to_lidar.transform.rotation.y,
            cam_to_lidar.transform.rotation.z
        );
        Eigen::Matrix3d rotation = q.toRotationMatrix();
        Eigen::Vector3d translation(
            cam_to_lidar.transform.translation.x,
            cam_to_lidar.transform.translation.y,
            cam_to_lidar.transform.translation.z
        );
        
        // カメラ座標系での点群データを作成し、z>0.1mでフィルタ
        std::vector<cv::Point3f> objectPoints;
        std::vector<size_t> valid_indices; // フィルタリング後のインデックス
        objectPoints.reserve(pointcloud.points.size());
        valid_indices.reserve(pointcloud.points.size());
        
        for (size_t i = 0; i < pointcloud.points.size(); ++i) {
            const auto &point = pointcloud.points[i];
            Eigen::Vector3d pt_lidar(point.x, point.y, point.z);
            Eigen::Vector3d pt_camera = rotation * pt_lidar + translation;
            
            // z > 0.1m のみ投影対象とする(Livoxはカメラに近い~0.13m)
            if (pt_camera.z() > 0.1) {
                // カメラ座標系に変換済みの点を追加（projectPointsにはrvec=0,tvec=0で渡すため）
                objectPoints.emplace_back(
                    static_cast<float>(pt_camera.x()),
                    static_cast<float>(pt_camera.y()),
                    static_cast<float>(pt_camera.z())
                );
                valid_indices.push_back(i);
            }
        }
        
        std::cerr<<"[project_image_coordinate] Input: "<<pointcloud.size()
                 <<", After z>0.1m filter: "<<objectPoints.size()<<std::endl;

        // プロジェクション結果を格納するベクトル
        std::vector<cv::Point2f> imagePoints;

        // 点群を画像平面にプロジェクション
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        // 結果をEigen形式に変換
        std::vector<Eigen::RowVectorXd> ret_rows;
        ret_rows.reserve(imagePoints.size());
        
        // デバッグ用: 投影座標の分布を確認
        int left_half_count = 0;  // x < 960
        int right_half_count = 0; // x >= 960
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::min();
        
        for (size_t i = 0; i < imagePoints.size(); ++i) {
            const auto &imgPt = imagePoints[i];
            const auto &point = pointcloud.points[valid_indices[i]];
            
            // 統計情報を収集
            if (imgPt.x < min_x) min_x = imgPt.x;
            if (imgPt.x > max_x) max_x = imgPt.x;

            // 画像内の点のみを追加
            if (0 <= std::round(imgPt.x) && std::round(imgPt.x) < caminfo.width &&
                0 <= std::round(imgPt.y) && std::round(imgPt.y) < caminfo.height) {
                
                // 左右の分布をカウント
                if (imgPt.x < 960) {
                    left_half_count++;
                } else {
                    right_half_count++;
                }
                
                Eigen::RowVectorXd row(5);
                row << imgPt.x, imgPt.y, point.x, point.y, point.z;
                ret_rows.push_back(row);
            }
        }
        
        std::cerr<<"[project_image_coordinate] Projected to image: "<<ret_rows.size()
                 <<" points (Left: "<<left_half_count<<", Right: "<<right_half_count
                 <<", x range: ["<<min_x<<", "<<max_x<<"])"<<std::endl;

        return ret_rows;
    }
    std::vector<Eigen::RowVectorXd> project_image_coordinate(
        const sensor_msgs::msg::CameraInfo &caminfo,
        const geometry_msgs::msg::TransformStamped &cam_to_lidar,
        const pcl::PointCloud<pcl::PointXYZ> &pointcloud
        ){
        auto start = std::chrono::high_resolution_clock::now();
        //create affine
        Eigen::Affine3d transform_matrix = Eigen::Affine3d::Identity();
        //rotation
        Eigen::Quaterniond rotation(cam_to_lidar.transform.rotation.w,
                                    cam_to_lidar.transform.rotation.x,
                                    cam_to_lidar.transform.rotation.y,
                                    cam_to_lidar.transform.rotation.z);
        //set rotation
        transform_matrix.rotate(rotation);
        //set translation
        transform_matrix.translation() <<   cam_to_lidar.transform.translation.x,
                                            cam_to_lidar.transform.translation.y,
                                            cam_to_lidar.transform.translation.z; 
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //std::cout << "It took" << elapsed_time <<"ms to create affine."<<std::endl;
        start = std::chrono::high_resolution_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //convert to camera coordinate
        pcl::transformPointCloud(pointcloud, *transformed_cloud,transform_matrix);
        //filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> filter;
        filter.setInputCloud(transformed_cloud);
        filter.setFilterFieldName("z");
        // Livoxはカメラに近い(~0.13m)ため、最小距離を0.1mに変更
        filter.setFilterLimits(0.1, std::numeric_limits<float>::max());
        filter.filter(*filtered_cloud);
        std::cerr<<"[Projection] Input points: "<<pointcloud.size()
                 <<", Transformed: "<<transformed_cloud->size()
                 <<", After z-filter(>0.1m): "<<filtered_cloud->size()<<std::endl;
        //std::cout<<"transfomed size:"<<transformed_cloud->size()<<std::endl;
        //std::cout<<"filtered size:"<<filtered_cloud->size()<<std::endl;

        end = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //std::cout << "It took" << elapsed_time <<"ms to transform."<<std::endl;
        //create 3,4 matrix for projection
        start = std::chrono::high_resolution_clock::now();
        Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> projection_mtx(caminfo.p.data());

        Eigen::MatrixXd points_matrix(filtered_cloud->size(), 4);
        for (size_t i = 0; i < filtered_cloud->size(); ++i) {
            points_matrix.row(i) << filtered_cloud->points[i].x,
                                    filtered_cloud->points[i].y,
                                    filtered_cloud->points[i].z,
                                    1.0;
        }
        Eigen::MatrixXd projected_points = (projection_mtx * points_matrix.transpose()).transpose();

        Eigen::ArrayXd z_inv = projected_points.col(2).array().inverse();
        Eigen::ArrayXd u = (projected_points.col(0).array() * z_inv).eval();
        Eigen::ArrayXd v = (projected_points.col(1).array() * z_inv).eval();

        Eigen::Array<bool, Eigen::Dynamic, 1> valid = 
            (u >= 0).array() && (u < caminfo.width).array() &&
            (v >= 0).array() && (v < caminfo.height).array();

        std::vector<Eigen::RowVectorXd> ret_rows;
        ret_rows.reserve(valid.count());
        for (int i = 0; i < valid.size(); ++i) {
            if (valid(i)) {
                Eigen::RowVectorXd ptXYD(5);
                ptXYD << u(i), v(i), points_matrix(i, 0), points_matrix(i, 1), points_matrix(i, 2);
                ret_rows.emplace_back(ptXYD);
            }
        }
        std::cerr<<"[Projection] Projected to image: "<<ret_rows.size()
                 <<" points (from "<<filtered_cloud->size()<<" filtered points)"<<std::endl;

        //for(const auto & point : filtered_cloud->points){
            //Eigen::Vector4d P_camera(point.x,point.y,point.z,1.0);
            ////project to image coordinate
            //Eigen::Vector3d P_image = projection_mtx*P_camera;
            //double u = P_image(0)/P_image(2);
            //double v = P_image(1)/P_image(2);
            ////calculate when point is inside of frame.
            //if( 0 <= std::round(u) && std::round(u) < caminfo.width &&
                //0 <= std::round(v) && std::round(v) < caminfo.height){
                //Eigen::RowVectorXd ptXYD(5);
                //ptXYD << u,v,point.x,point.y,point.z;
                //ret_rows.push_back(ptXYD);
            //}
        //}
        end = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "It took" << elapsed_time <<"ms to project."<<std::endl;
        return ret_rows;
    }


    vehicle_interface::msg::ProjectedPoints create_projected_msg(
        const std::vector<Eigen::RowVectorXd> &projected_points
    ){
        vehicle_interface::msg::ProjectedPoints msg;

        for(const Eigen::RowVectorXd &point : projected_points){
            vehicle_interface::msg::PointImage pt;
            pt.u = point(0);
            pt.v = point(1);
            pt.x = point(2);
            pt.y = point(3);
            pt.z = point(4);
            msg.points.push_back(pt);
        }

        return msg;
    }
}// namespace nedo_localization