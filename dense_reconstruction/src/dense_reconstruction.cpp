#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

Eigen::Matrix4f T_wc = Eigen::Matrix4f::Identity(); // 存储相机位姿

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Eigen::Quaternionf q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    Eigen::Matrix3f R = q.toRotationMatrix();
    
    T_wc.block<3,3>(0,0) = R;
    T_wc(0,3) = msg->pose.position.x;
    T_wc(1,3) = msg->pose.position.y;
    T_wc(2,3) = msg->pose.position.z;

    ROS_INFO("Updated ORBSLAM3 Pose.");
}

pcl::PointCloud<pcl::PointXYZ> sparse_map;

void sparseCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, sparse_map);
    ROS_INFO("Received sparse map with %lu points", sparse_map.size());
}

class DenseReconstruction {
public:
    DenseReconstruction() {
        // 订阅双目图像
        left_image_sub_ = nh_.subscribe("/camera/fisheye1/image_raw", 1, &DenseReconstruction::leftImageCallback, this);
        right_image_sub_ = nh_.subscribe("/camera/fisheye2/image_raw", 1, &DenseReconstruction::rightImageCallback, this);

        // 发布稠密点云
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dense_point_cloud", 1);

        // 初始化SGBM
        stereo_ = cv::StereoSGBM::create(0, 64, 11, 8 * 3 * 11 * 11, 32 * 3 * 11 * 11, 1, 10, 100, 32);
    }

private:
    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            left_image_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            right_image_ = cv_ptr->image;
            if (!left_image_.empty()) {
                processImages();
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void processImages() {
        // 计算视差图
        cv::Mat disparity;
        stereo_->compute(left_image_, right_image_, disparity);
    
        // 生成深度图
        float focal_length = 0.8;  // 相机焦距（单位：米）
        float baseline = 0.1;      // 双目基线（单位：米）
        cv::Mat depth = (focal_length * baseline) / (disparity / 16.0f);
    
        // 设定尺度因子，确保单位合理
        float scale_factor = 100;  // 调整到合理范围
    
        // 生成点云
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (int v = 0; v < depth.rows; ++v) {
            for (int u = 0; u < depth.cols; ++u) {
                float z = depth.at<float>(v, u);
                if (z > 0 && z < 100.0) {  // 过滤掉无效点
                    float x = (u - depth.cols / 2) * z / focal_length;
                    float y = (v - depth.rows / 2) * z / focal_length;
                    z *= scale_factor;  // 统一放大
    
                    // 转换到全局坐标
                    Eigen::Vector4f P_local(x, y, z, 1.0);
                    Eigen::Vector4f P_global = T_wc * P_local;
    
                    cloud.push_back(pcl::PointXYZ(P_global[0], P_global[1], P_global[2]));
                }
            }
        }
    
        // **检查 SGBM 生成的点是否接近 ORBSLAM3 稀疏点**
        for (auto& point : cloud) {
            for (auto& sparse_point : sparse_map) {
                float dist = sqrt(pow(point.x - sparse_point.x, 2) +
                                  pow(point.y - sparse_point.y, 2) +
                                  pow(point.z - sparse_point.z, 2));
    
                if (dist < 0.05) { // 若误差较小，则融合
                    point.z = (point.z + sparse_point.z) / 2.0;  
                }
            }
        }
    
        // 发布点云
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";  // 使用 ORBSLAM3 的全局坐标系
        point_cloud_pub_.publish(cloud_msg);
    
        // 调试输出
        ROS_INFO("Generated point cloud with %lu points", cloud.size());
    }    

    ros::NodeHandle nh_;
    ros::Subscriber left_image_sub_;
    ros::Subscriber right_image_sub_;
    ros::Subscriber pose_sub = nh_.subscribe("/orb_slam3/pose", 10, poseCallback);
    ros::Subscriber sparse_cloud_sub = nh_.subscribe("/orb_slam3/map_points", 10, sparseCloudCallback);
    
    ros::Publisher point_cloud_pub_;
    cv::Mat left_image_;
    cv::Mat right_image_;
    cv::Ptr<cv::StereoSGBM> stereo_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dense_reconstruction");
    DenseReconstruction node;
    ros::spin();
    return 0;
}
