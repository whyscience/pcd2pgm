#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>  // 直通滤波器头文件
//#include <pcl/filters/voxel_grid.h>   // 体素滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>  // 半径滤波器头文件

#include <string>
//#include <opencv2/opencv.hpp>
#include <random>


class PCLFiltersNode : public rclcpp::Node {
public:
    PCLFiltersNode() : Node("pcl_filters"), pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                       cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>),
                       cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>) {
        this->declare_parameter<std::string>("file_directory",
                                             "/home/eric/Desktop/");
        this->declare_parameter<std::string>("file_name", "result");
        this->declare_parameter<double>("thre_z_min", 0.4);
        this->declare_parameter<double>("thre_z_max", 0.5);
        this->declare_parameter<int>("flag_pass_through", 0);
        this->declare_parameter<double>("thre_radius", 0.5);
        this->declare_parameter<double>("map_resolution", 0.05);
        this->declare_parameter<std::string>("map_topic_name", "map");

        this->get_parameter("file_directory", file_directory);
        this->get_parameter("file_name", file_name);
        this->get_parameter("thre_z_min", thre_z_min);
        this->get_parameter("thre_z_max", thre_z_max);
        this->get_parameter("flag_pass_through", flag_pass_through);
        this->get_parameter("thre_radius", thre_radius);
        this->get_parameter("map_resolution", map_resolution);
        this->get_parameter("map_topic_name", map_topic_name);
        pcd_file = file_directory + file_name + ".pcd";

        RCLCPP_INFO(this->get_logger(), "*** file_directory = %s ***", file_directory.c_str());
        RCLCPP_INFO(this->get_logger(), "*** file_name = %s ***", file_name.c_str());
        RCLCPP_INFO(this->get_logger(), "*** pcd_file = %s ***", pcd_file.c_str());

        map_topic_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

        //subscribe to the point cloud topic and run the filter
        point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/Laser_map", 10, std::bind(&PCLFiltersNode::point_cloud_callback, this, std::placeholders::_1));

        //create a map_pub_timer to publish the map
        //map_pub_timer = this->create_wall_timer(std::chrono::milliseconds(1000),
        //                                        std::bind(&PCLFiltersNode::publish_map, this));

        //load_pcd_file();
        //run_filter();
    }

private:
    std::string file_directory;
    std::string file_name;
    std::string pcd_file;

    std::string map_topic_name;
    const std::string pcd_format = ".pcd";
    nav_msgs::msg::OccupancyGrid map_topic_msg;

    float thre_z_min = 0.3;
    float thre_z_max = 0.6;
    int flag_pass_through = 0;
    double map_resolution = 0.05;
    double thre_radius = 0.1;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_topic_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
    rclcpp::TimerBase::SharedPtr map_pub_timer; // 定时器的句柄
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough;

private:

    void load_pcd_file() {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "初始点云数据点数：%zu", pcd_cloud->points.size());
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Received point cloud message");
        pcl::fromROSMsg(*msg, *pcd_cloud);
        run_filter();
        publish_map();
    }

    void run_filter() {
        PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));
        //   RadiusOutlierFilter(cloud_after_PassThrough, 0.1, 10);
        //   SetMapTopicMsg(cloud_after_Radius, map_topic_msg);

        SetMapTopicMsg(cloud_after_PassThrough, map_topic_msg);
    }

    void publish_map() {
        //if (map_topic_pub->get_subscription_count() > 0) {
        if (!map_topic_msg.data.empty()) {
            map_topic_pub->publish(map_topic_msg);
            //publishOccupancyGrid();
            //std::cout << "map_topic_msg.data.size() = " << map_topic_msg.data.size() << std::endl;
        }
        //}
    }

    // Define other member variables and methods for filters and processing...
    void PassThroughFilter(const float &thre_low, const float &thre_high, const bool &flag_in) {
        /*方法一：直通滤波器对点云进行处理。*/
        pcl::PassThrough<pcl::PointXYZ> passthrough;
        passthrough.setInputCloud(pcd_cloud);//输入点云
        passthrough.setFilterFieldName("z");//对z轴进行操作
        passthrough.setFilterLimits(thre_low, thre_high);//设置直通滤波器操作范围
        passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
        passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
        RCLCPP_INFO(rclcpp::get_logger("pcl"), "直通滤波后点云数据点数：%zu", cloud_after_PassThrough->points.size());
    }

    void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0, const double &radius,
                             const int &thre_count) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierRemoval; // 创建滤波器

        radiusOutlierRemoval.setInputCloud(pcd_cloud0); // 设置输入点云
        radiusOutlierRemoval.setRadiusSearch(radius); // 设置radius为指定范围内找临近点
        radiusOutlierRemoval.setMinNeighborsInRadius(thre_count); // 设置查询点的邻域点集数小于指定值的删除

        radiusOutlierRemoval.filter(*cloud_after_Radius);
        RCLCPP_INFO(rclcpp::get_logger("pcl"), "半径滤波后点云数据点数：%zu", cloud_after_Radius->points.size());
    }

    void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::msg::OccupancyGrid &msg) {
        //msg.header.seq = 0;
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = "map";

        float x_min, x_max, y_min, y_max;

        if (cloud->points.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("pcl"), "pcd is empty!");
            return;
        }

        // 计算点云边界
        for (unsigned int i = 0; i < cloud->points.size() - 1; i++) {
            if (i == 0) {
                x_min = x_max = cloud->points[i].x;
                y_min = y_max = cloud->points[i].y;
            }

            float x = cloud->points[i].x;
            float y = cloud->points[i].y;

            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;

            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
        //std::cout << "x_min = " << x_min << ", x_max = " << x_max << std::endl;

        msg.info.origin.position.x = x_min;
        msg.info.origin.position.y = y_min;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.info.resolution = map_resolution;
        msg.info.width = static_cast<uint32_t>((x_max - x_min) / map_resolution);
        msg.info.height = static_cast<uint32_t>((y_max - y_min) / map_resolution);

        msg.data.resize(msg.info.width * msg.info.height, -1); // Initialize with -1 for unknown
        msg.data.assign(msg.info.width * msg.info.height, 0);

        // 点云到栅格地图的转换略去，根据实际需要填充msg.data
        int count = 0;
        for (auto &point: cloud->points) {
            int i = int((point.x - x_min) / map_resolution);
            if (i < 0 || i >= (int) msg.info.width) continue;

            int j = int((point.y - y_min) / map_resolution);
            if (j < 0 || j >= (int) msg.info.height - 1) continue;

            msg.data[i + j * msg.info.width] = 100;
            //msg.data[i + j * msg.info.width] = int(255 * (cloud->points[iter].z * k_line + b_line)) % 255;
            //std::cout << "(" << i << "," << j << "), ";
            count++;
        }
        //std::cout << "msg.info.width = " << msg.info.width << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("pcl"), "data size = %zu, valid = %d", msg.data.size(), count);
        //visualizeOccupancyGrid(msg);
    }

    void publishOccupancyGrid() {
        auto message = nav_msgs::msg::OccupancyGrid();
        // 填充header信息
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "map";

        // 填充地图信息，例如地图分辨率、宽度、高度等
        message.info.resolution = 0.1; // 假设每个单元格代表0.1m
        message.info.width = 100;       // 100单元格宽
        message.info.height = 100;      // 100单元格高
        // 设置地图的原点，通常是在地图的左下角
        message.info.origin.position.x = 0.0;
        message.info.origin.position.y = 0.0;
        message.info.origin.position.z = 0.0;
        message.info.origin.orientation.w = 1.0;

        // 填充地图数据，这里以0填充表示所有区域都是自由的
        message.data.resize(message.info.width * message.info.height, 0);

        // 随机填充地图数据
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(-1, 100);
        for (auto &value: message.data) {
            int dice_roll = distribution(generator); // 生成-1到100之间的随机数
            // 将随机值归一化到-1, 0, 100
            if (dice_roll < 33) {
                value = -1; // 未知区域
            } else if (dice_roll < 66) {
                value = 0;  // 自由区域
            } else {
                value = 100; // 占用区域
            }
        }

        map_topic_pub->publish(message);
    }

    /*void visualizeOccupancyGrid(const nav_msgs::msg::OccupancyGrid& msg) {
        int width = msg.info.width;
        int height = msg.info.height;
        auto map_data = msg.data;

        // 创建一个黑白图像，每个像素对应一个栅格
        cv::Mat img(height, width, CV_8UC1);

        // 填充图像
        for(int i = 0; i < height; ++i) {
            for(int j = 0; j < width; ++j) {
                int index = i * width + j;
                if(map_data[index] == 0) { // Free space
                    img.at<uchar>(i, j) = 255; // 白色
                } else if(map_data[index] == 100) { // Occupied space
                    img.at<uchar>(i, j) = 0; // 黑色
                } else { // Unknown space
                    img.at<uchar>(i, j) = 127; // 灰色
                }
            }
        }

        // 显示图像
        cv::imshow("OccupancyGrid", img);
        cv::waitKey(1000); // 等待用户按键
    }*/
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLFiltersNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
