#include <geometry_msgs/msg/pose_array.hpp>
#include <iomanip>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

static constexpr int8_t OBSTACLE_THRESHOLD = 65; // 障碍物阈值

class ObstacleExtractor : public rclcpp::Node
{
public:
    ObstacleExtractor()
        : Node("obstacle_extractor"), clear_count_(0)
    {
        // 声明动态参数（含区域过滤参数）
        this->declare_parameter("costmap_topic", "/local_costmap/costmap");
        this->declare_parameter("region_origin_x", 0.0); // 区域原点X
        this->declare_parameter("region_origin_y", 0.0); // 区域原点Y
        this->declare_parameter("region_xlength", 14.5); // X方向长度（可正可负）
        this->declare_parameter("region_ylength", -7.0); // Y方向长度（可正可负）
        update_parameters();

        // 初始化订阅器和发布器
        recreate_subscription();
        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/global_obstacles", 10);

        // 注册参数回调
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleExtractor::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "节点已启动，监听话题: %s", costmap_topic_.c_str());
    }

private:
    void update_parameters()
    {
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
        region_origin_x_ = this->get_parameter("region_origin_x").as_double();
        region_origin_y_ = this->get_parameter("region_origin_y").as_double();
        region_xlength_ = this->get_parameter("region_xlength").as_double();
        region_ylength_ = this->get_parameter("region_ylength").as_double();

        recreate_subscription();
    }

    void recreate_subscription()
    {
        if (costmap_sub_)
        {
            costmap_sub_.reset(); // 先释放旧订阅
        }
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 10,
            std::bind(&ObstacleExtractor::costmap_callback, this, std::placeholders::_1));
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        geometry_msgs::msg::PoseArray obstacles;
        obstacles.header = msg->header;

        const auto &data = msg->data;
        const double res = msg->info.resolution;
        const double ox = msg->info.origin.position.x;
        const double oy = msg->info.origin.position.y;
        const uint32_t width = msg->info.width;
        const uint32_t height = msg->info.height;

        // 根据参数计算有效的矩形区域 [支持负值区域]-----------------
        const double region_x_min = (region_xlength_ >= 0) ? region_origin_x_ : region_origin_x_ + region_xlength_;
        const double region_x_max = (region_xlength_ >= 0) ? region_origin_x_ + region_xlength_ : region_origin_x_;
        const double region_y_min = (region_ylength_ >= 0) ? region_origin_y_ : region_origin_y_ + region_ylength_;
        const double region_y_max = (region_ylength_ >= 0) ? region_origin_y_ + region_ylength_ : region_origin_y_;
        // -----------------------------------------------------

        bool has_obstacles = false;
        for (uint32_t idx = 0; idx < data.size(); ++idx)
        {
            if (data[idx] > OBSTACLE_THRESHOLD && data[idx] != -1)
            {
                const uint32_t i = idx / width; // 行索引
                const uint32_t j = idx % width; // 列索引

                // 计算地图坐标系下的坐标
                const double x = ox + (j + 0.5) * res;
                const double y = oy + (i + 0.5) * res;

                // 区域过滤检查 [支持任意方向矩形区域]
                if (x >= region_x_min && x <= region_x_max &&
                    y >= region_y_min && y <= region_y_max)
                {
                    geometry_msgs::msg::Pose p;
                    p.position.x = x;
                    p.position.y = y;
                    obstacles.poses.push_back(p);
                    has_obstacles = true;
                }
            }
        }

        // 障碍物计数与发布逻辑
        if (has_obstacles)
        {
            clear_count_ = 0;
            obstacle_pub_->publish(obstacles);
            print_obstacle_info(obstacles, res, width, height);
        }
        else
        {
            if (++clear_count_ >= 5)
            {
                geometry_msgs::msg::PoseArray empty_obstacles;
                empty_obstacles.header.stamp = this->now();
                empty_obstacles.header.frame_id = msg->header.frame_id;
                obstacle_pub_->publish(empty_obstacles);
                RCLCPP_WARN(this->get_logger(), "连续5次未检测到障碍物，已清空障碍点");
                clear_count_ = 0;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "未检测到障碍物 (%d/5)", clear_count_);
            }
        }
    }

    void print_obstacle_info(const geometry_msgs::msg::PoseArray &obstacles,
                             double resolution, uint32_t width, uint32_t height)
    {

        // 仅当障碍点较少时打印详细信息
        if (obstacles.poses.size() <= 20)
        {
            for (size_t i = 0; i < obstacles.poses.size(); ++i)
            {
                const auto &pose = obstacles.poses[i];
            }
        }
    }

    // 参数回调函数（支持动态更新区域）
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        bool need_update_sub = false;
        bool need_log_region = false;

        for (const auto &param : params)
        {
            if (param.get_name() == "costmap_topic")
            {
                costmap_topic_ = param.as_string();
                need_update_sub = true;
                RCLCPP_INFO(this->get_logger(), "更新话题: %s", costmap_topic_.c_str());
            }
            else if (param.get_name() == "region_origin_x")
            {
                region_origin_x_ = param.as_double();
                need_log_region = true;
                RCLCPP_INFO(this->get_logger(), "更新区域原点X: %.2f", region_origin_x_);
            }
            else if (param.get_name() == "region_origin_y")
            {
                region_origin_y_ = param.as_double();
                need_log_region = true;
                RCLCPP_INFO(this->get_logger(), "更新区域原点Y: %.2f", region_origin_y_);
            }
            else if (param.get_name() == "region_xlength")
            {
                region_xlength_ = param.as_double();
                need_log_region = true;
                RCLCPP_INFO(this->get_logger(), "更新X长度: %.2f (方向: %s)",
                            region_xlength_,
                            (region_xlength_ >= 0) ? "正方向" : "负方向");
            }
            else if (param.get_name() == "region_ylength")
            {
                region_ylength_ = param.as_double();
                need_log_region = true;
                RCLCPP_INFO(this->get_logger(), "更新Y长度: %.2f (方向: %s)",
                            region_ylength_,
                            (region_ylength_ >= 0) ? "正方向" : "负方向");
            }
        }

        // 更新后打印区域信息
        if (need_log_region)
        {
            const double x_min = (region_xlength_ >= 0) ? region_origin_x_ : region_origin_x_ + region_xlength_;
            const double x_max = (region_xlength_ >= 0) ? region_origin_x_ + region_xlength_ : region_origin_x_;
            const double y_min = (region_ylength_ >= 0) ? region_origin_y_ : region_origin_y_ + region_ylength_;
            const double y_max = (region_ylength_ >= 0) ? region_origin_y_ + region_ylength_ : region_origin_y_;

            RCLCPP_INFO(this->get_logger(), "当前有效区域: X(%.2f ~ %.2f), Y(%.2f ~ %.2f)",
                        x_min, x_max, y_min, y_max);
        }

        if (need_update_sub)
        {
            recreate_subscription();
        }

        return result;
    }

    // 成员变量
    std::string costmap_topic_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    int clear_count_;

    // 区域过滤参数 (支持负值)
    double region_origin_x_;
    double region_origin_y_;
    double region_xlength_; // 可正可负，表示延伸方向和长度
    double region_ylength_; // 可正可负，表示延伸方向和长度
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}