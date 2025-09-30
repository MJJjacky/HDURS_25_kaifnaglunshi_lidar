#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <cmath>

class LidarProcessor
{
public:
    LidarProcessor()
    {
        // 初始化 ROS 节点句柄
        ros::NodeHandle nh;

        // 订阅激光雷达的 LaserScan 话题
        sub_ = nh.subscribe("/scan", 10, &LidarProcessor::laserScanCallback, this);

        // 发布经过半径滤波处理后的 LaserScan 数据
        pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan1", 10);

        // 设置滤波参数
        radius_ = 0.1;       // 半径
        min_neighbors_ = 5;  // 最小邻居数
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // 将 LaserScan 转换为 PointCloud2
        sensor_msgs::PointCloud2 cloud_msg;
        laserScanToPointCloud(msg, cloud_msg);

        // 将 PointCloud2 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(cloud_msg, *cloud_input);

        // 存储滤波后的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

        // 执行半径滤波
        RadiusOutlierFilter(cloud_input, cloud_filtered, radius_, min_neighbors_);

        // 将滤波后的点云转换回 LaserScan
        sensor_msgs::LaserScan filtered_laser_scan;
        pointCloudToLaserScan(cloud_filtered, msg, filtered_laser_scan);

        // 发布处理后的 LaserScan 数据
        pub_.publish(filtered_laser_scan);
        // ROS_INFO("Published filtered LaserScan with %lu ranges", filtered_laser_scan.ranges.size());
    }

private:
    // 半径滤波函数
    void RadiusOutlierFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filter,
                             float radius, int num)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem; // 创建滤波器
        outrem.setInputCloud(cloud_input);               // 设置输入点云
        outrem.setRadiusSearch(radius);                  // 设置搜索半径
        outrem.setMinNeighborsInRadius(num);             // 设置最小邻居数
        outrem.filter(*cloud_filter);                    // 执行滤波
    }

    // LaserScan 转换为 PointCloud2
    void laserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan, sensor_msgs::PointCloud2 &cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXYZI point;
                point.x = range * std::cos(angle);
                point.y = range * std::sin(angle);
                point.z = 0.0;
                point.intensity = 0.0; // Intensity 默认为 0
                pcl_cloud->points.push_back(point);
            }
        }
        pcl::toROSMsg(*pcl_cloud, cloud);
        cloud.header = scan->header;
    }

    // PointCloud2 转换为 LaserScan
    void pointCloudToLaserScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                               const sensor_msgs::LaserScan::ConstPtr &scan,
                               sensor_msgs::LaserScan &laser_scan)
    {
        laser_scan = *scan; // 复制原始 LaserScan 的元信息
        std::fill(laser_scan.ranges.begin(), laser_scan.ranges.end(), std::numeric_limits<float>::infinity());

        for (const auto &point : cloud->points)
        {
            float angle = std::atan2(point.y, point.x);
            float range = std::sqrt(point.x * point.x + point.y * point.y);

            // 检查点是否在激光雷达的扫描范围内
            if (angle >= scan->angle_min && angle <= scan->angle_max)
            {
                int index = std::round((angle - scan->angle_min) / scan->angle_increment);
                if (index >= 0 && index < static_cast<int>(laser_scan.ranges.size()))
                {
                    laser_scan.ranges[index] = std::min(laser_scan.ranges[index], range);
                }
            }
        }
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    float radius_;
    int min_neighbors_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "lidar_processor");

    // 创建处理对象
    LidarProcessor processor;

    // ROS 主循环
    ros::spin();

    return 0;
}
