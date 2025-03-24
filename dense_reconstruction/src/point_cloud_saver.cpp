#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <signal.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_set>

// 定义哈希函数，用于存储 pcl::PointXYZ
struct PointHash {
    size_t operator()(const pcl::PointXYZ& p) const {
        return std::hash<float>()(p.x) ^ std::hash<float>()(p.y) ^ std::hash<float>()(p.z);
    }
};

// 定义相等比较函数
struct PointEqual {
    bool operator()(const pcl::PointXYZ& a, const pcl::PointXYZ& b) const {
        return (fabs(a.x - b.x) < 1e-6) && 
               (fabs(a.y - b.y) < 1e-6) && 
               (fabs(a.z - b.z) < 1e-6);
    }
};

class PointCloudSaver {
public:
    PointCloudSaver() {
        point_cloud_sub_ = nh_.subscribe("/dense_point_cloud", 1, &PointCloudSaver::pointCloudCallback, this);
        save_triggered_ = false;

        // 监听 SIGINT (Ctrl+C) 信号，确保程序退出时保存点云
        signal(SIGINT, signalHandler);
    }

    ~PointCloudSaver() {
        savePointCloud();  // 退出时保存最终点云
    }

    static void signalHandler(int signum) {
        ROS_WARN("Interrupt signal received, saving point cloud before exit...");
        instance_->savePointCloud();
        ros::shutdown(); // 终止 ROS 进程
    }

    static PointCloudSaver* instance_;  // 用于处理信号的静态实例指针

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        pcl::PointCloud<pcl::PointXYZ> new_cloud;
        pcl::fromROSMsg(*msg, new_cloud);

        // 追加新点云
        cloud_ += new_cloud;

        ROS_INFO("Accumulated point cloud size: %lu points", cloud_.size());

        // 条件触发保存（如果累计点数过大，则提前保存）
        if (cloud_.size() > 50000 && !save_triggered_) { 
            savePointCloud();
            save_triggered_ = true;  // 只保存一次
        }
    }

    void savePointCloud() {
        std::lock_guard<std::mutex> lock(mutex_);
    
        if (cloud_.empty()) {
            ROS_WARN("No points in cloud, skipping save.");
            return;
        }
    
        // 1️⃣ **使用哈希表去重**
        std::unordered_set<pcl::PointXYZ, PointHash, PointEqual> unique_points;
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    
        for (const auto& point : cloud_) {
            if (unique_points.insert(point).second) {  // 插入成功说明该点是新点
                filtered_cloud.push_back(point);
            }
        }
    
        ROS_INFO("Removed duplicate points, %lu unique points remain.", filtered_cloud.size());
    
        // 2️⃣ **应用体素网格滤波，进一步去除冗余点**
        pcl::PointCloud<pcl::PointXYZ> voxel_filtered_cloud;
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(filtered_cloud.makeShared());
        voxel.setLeafSize(0.05f, 0.05f, 0.05f);  // 体素尺寸，值越大去重越强
        voxel.filter(voxel_filtered_cloud);
    
        ROS_INFO("After VoxelGrid filtering, %lu points remain.", voxel_filtered_cloud.size());
    
        // 3️⃣ **保存最终优化的点云**
        pcl::io::savePCDFileASCII("dense_test.pcd", cloud_);
        ROS_INFO("Final point cloud saved to dense_map.pcd with %lu points", cloud_.size());
    }

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    std::mutex mutex_;
    bool save_triggered_;  // 避免多次保存
};

PointCloudSaver* PointCloudSaver::instance_ = nullptr;

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_saver");

    PointCloudSaver saver;
    PointCloudSaver::instance_ = &saver; // 设置静态实例

    ros::Rate rate(10);  // 控制循环频率，避免 spin() 无限占用资源
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

