#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from tf2_ros import Buffer, TransformException, TransformListener  # 新增TF2相关导入


class OptimalPointSelector(Node):
    def __init__(self):
        super().__init__('optimal_point_selector')
        
        # 声明动态参数
        self.declare_parameter('center_x', 3.5, 
                              ParameterDescriptor(description='圆心X坐标', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('center_y', -14.0,
                              ParameterDescriptor(description='圆心Y坐标', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('refer_point_x', 3.5,
                              ParameterDescriptor(description='参考点X坐标', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('refer_point_y', 0.0,
                              ParameterDescriptor(description='参考点Y坐标', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('comd_r', 2.5,
                              ParameterDescriptor(description='指令半径', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('a', 1.0,
                              ParameterDescriptor(description='障碍物评分权重', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('b', 2.0,
                              ParameterDescriptor(description='角度评分权重', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('c', 5.0,
                              ParameterDescriptor(description='半径匹配评分权重', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('d', 1.0, 
                              ParameterDescriptor(description='机器人距离评分权重', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('max_diff', 3.0,
                              ParameterDescriptor(description='最大允许半径差值', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('d', 1.0,
                              ParameterDescriptor(description='机器人距离评分参数', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('map_frame', 'map',
                              ParameterDescriptor(description='地图坐标系', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('robot_frame', 'base_link',
                              ParameterDescriptor(description='机器人坐标系', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('dist_decay', 1.5,
                              ParameterDescriptor(description='距离衰减因子', type=ParameterType.PARAMETER_DOUBLE))
        
        self.points_sub = self.create_subscription(
            PoseArray,
            '/points_select',
            self.points_callback,
            10)
        self.get_logger().info("已订阅候选点话题: /points_select")
        
        self.obstacle_sub = self.create_subscription(
            PoseArray,
            '/global_obstacles',
            self.obstacle_callback,
            10)
        self.get_logger().info("已订阅障碍物话题: /global_obstacles")
        
        # 发布最优点
        self.best_point_pub = self.create_publisher(
            PoseStamped,
            '/optimal_point',
            10)
        self.get_logger().info("已创建最优点点话题: /optimal_point")
        
        # 新增：创建Point格式的最优点发布者
        self.optimal_point_data_pub = self.create_publisher(
            Point,
            '/optimal_point_data',
            10)
        self.get_logger().info("已创建Point格式的最优点话题: /optimal_point_data")
        
        # 新增：订阅最优点的订阅者
        self.optimal_point_sub = self.create_subscription(
            PoseStamped,
            '/optimal_point',
            self.optimal_point_callback,
            10)
        self.get_logger().info("已订阅最优点话题: /optimal_point")
        
        # 存储障碍物点
        self.obstacle_points = []
        self.obstacle_data_received = False  # 新增：标记是否收到障碍物数据
        

        # 新增：TF2相关初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF2监听器已初始化")

        
        # 参数变更回调
        self.add_on_set_parameters_callback(self.param_callback)
        self.get_logger().info("节点初始化完成，等待数据...")
    
    def update_robot_position(self):
        """定期更新机器人位置并打印坐标"""
        try:
            # 获取坐标系参数
            map_frame = self.get_parameter('map_frame').value
            robot_frame = self.get_parameter('robot_frame').value
            
            # 查询最新变换
            transform = self.tf_buffer.lookup_transform(
                map_frame, 
                robot_frame, 
                rclpy.time.Time(),
                timeout=rclpy.time.Duration(seconds=0.1)
            )
            
            # 更新机器人位置
            self.current_robot_position.x = transform.transform.translation.x
            self.current_robot_position.y = transform.transform.translation.y
            self.robot_position_valid = True
            self.robot_position_time = self.get_clock().now()
            
            # 打印机器人位置（限流输出）
            self.get_logger().info(
                f"📍 机器人定位坐标: x={self.current_robot_position.x:.2f}m, y={self.current_robot_position.y:.2f}m",
                throttle_duration_sec=1.0
            )
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            # 检查位置是否过期（超过1秒）
            time_diff = (self.get_clock().now() - self.robot_position_time).nanoseconds / 1e9
            if time_diff > 1.0:
                self.robot_position_valid = False
                self.get_logger().error(f"🚨 TF查询失败: {str(e)}", throttle_duration_sec=1.0)
        except Exception as e:
            self.robot_position_valid = False
            self.get_logger().error(f"位置更新异常: {str(e)}", throttle_duration_sec=1.0)
    
    def param_callback(self, params):
        """参数变更回调函数"""
        updated_params = []
        for param in params:
            updated_params.append(f"{param.name}={param.value}")
            self.get_logger().info(f"参数更新: {param.name} = {param.value}")
        
        self.get_logger().info(f"更新参数列表: {', '.join(updated_params)}")
        return SetParametersResult(successful=True)
    
    def obstacle_callback(self, msg):
        """障碍物数据回调"""
        self.obstacle_points = [pose.position for pose in msg.poses]
        self.obstacle_data_received = True  # 标记已收到障碍物数据
        

    
    def optimal_point_callback(self, msg):
        """最优点的回调函数 - 转换为Point格式发布"""
        point_msg = Point()
        point_msg.x = msg.pose.position.x
        point_msg.y = msg.pose.position.y
        point_msg.z = msg.pose.position.z
        
        self.optimal_point_data_pub.publish(point_msg)

    
    def points_callback(self, msg):
        """候选点数据回调 - 即使TF查询失败也继续计算"""
        if not msg.poses:
            self.get_logger().warn("收到空候选点列表，跳过处理")
            return
        
        self.get_logger().info(f"收到 {len(msg.poses)} 个候选点")
        
        # 打印前3个候选点位置
        if len(msg.poses) > 0:
            points_info = []
            for i, pose in enumerate(msg.poses[:3]):
                points_info.append(f"点{i+1}: ({pose.position.x:.2f}, {pose.position.y:.2f})")
            self.get_logger().info(f"候选点位置(前3个): {'; '.join(points_info)}")
            
        # 获取当前参数值
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        refer_x = self.get_parameter('refer_point_x').value
        refer_y = self.get_parameter('refer_point_y').value
        comd_r = self.get_parameter('comd_r').value
        a = self.get_parameter('a').value
        b = self.get_parameter('b').value
        c = self.get_parameter('c').value
        d = self.get_parameter('d').value
        max_diff = self.get_parameter('max_diff').value

        # 新增距离评分参数
        d = self.get_parameter('d').value
        map_frame = self.get_parameter('map_frame').value
        robot_frame = self.get_parameter('robot_frame').value
        dist_decay = self.get_parameter('dist_decay').value

        
        center = Point(x=center_x, y=center_y)
        refer = Point(x=refer_x, y=refer_y)
        
        # 计算每个点的原始得分
        raw_obstacle_scores = []
        raw_angle_scores = []

        # 新增：距离得分
        raw_distance_scores = []
        has_tf = True  # 标记是否成功获取TF变换

        
        self.get_logger().info("开始计算候选点得分...")
        
        # 新增：尝试获取机器人当前位置（非阻塞式）
        robot_position = None
        try:
            # 获取最新的变换（非阻塞）
            transform = self.tf_buffer.lookup_transform(
                map_frame, robot_frame, rclpy.time.Time())
            robot_position = Point()
            robot_position.x = transform.transform.translation.x
            robot_position.y = transform.transform.translation.y
            self.get_logger().info(f"成功获取机器人位置: ({robot_position.x:.2f}, {robot_position.y:.2f})")
        except TransformException as ex:
            self.get_logger().warn(f"无法获取机器人位置: {ex}")
            has_tf = False
        
        # 检查是否收到障碍物数据
        if not self.obstacle_data_received or not self.obstacle_points:
            self.get_logger().warn("⚠️ 未收到障碍物数据或障碍物列表为空，障碍物得分设为满分")
            # 如果没有障碍物数据，为所有点设置相同的最大障碍物距离得分
            raw_obstacle_scores = [10.0] * len(msg.poses)
        else:
            for pose in msg.poses:
                # 障碍物距离评分
                obs_score = self.calculate_obstacle_score(pose.position, self.obstacle_points)
                raw_obstacle_scores.append(obs_score)
        
        for pose in msg.poses:
            # 角度偏离评分
            angle_score = self.calculate_angle_score(pose.position, center, refer)
            raw_angle_scores.append(angle_score)
            

            # 新增：计算距离得分（如果成功获取机器人位置）
            if has_tf and robot_position:
                dist = self.calculate_distance(pose.position, robot_position)
                # 使用衰减因子法计算距离得分（距离越近得分越高）
                distance_score = math.exp(-dist * dist_decay)
                raw_distance_scores.append(distance_score)
            else:
                raw_distance_scores.append(0.0)

        
        # 半径匹配评分（所有点相同）
        actual_dist = self.calculate_distance(center, refer)
        radius_diff = abs(actual_dist - comd_r)
        radius_score = min(radius_diff, max_diff)  # 限制在最大差值范围内
        
        
        # 归一化处理
        if not self.obstacle_data_received or not self.obstacle_points:
            # 如果没有障碍物数据，归一化障碍物得分为全1
            norm_obstacle = [1.0] * len(msg.poses)
            self.get_logger().info("无障碍物数据，障碍物归一化得分设置为全1")
        else:
            norm_obstacle = self.normalize_scores(raw_obstacle_scores, higher_better=True)
        
        norm_angle = [1 - (angle/90) for angle in raw_angle_scores]  # 角度越小越好
        norm_radius = 1 - (radius_score / max_diff)  # 差值越小越好
        

        
        # 计算综合得分（新增d * norm_robot）
        total_scores = []
        for i in range(len(msg.poses)):
            total = (a * norm_obstacle[i] + 
                     b * norm_angle[i] + 
                     c * norm_radius +
                     d * raw_distance_scores[i])  # 新增距离评分项

            total_scores.append(total)
        
        # 找到最佳点
        best_index = np.argmax(total_scores)
        best_point = msg.poses[best_index]
        best_score = total_scores[best_index]
        
        # 发布最优点
        best_msg = PoseStamped()
        best_msg.header = msg.header
        best_msg.pose = best_point
        self.best_point_pub.publish(best_msg)
        
        # 记录日志 - 包含所有得分细节（新增距离评分项）
        log_message = (
            f"🔝 选出最优点: ({best_point.position.x:.2f}, {best_point.position.y:.2f}) "
            f"综合得分: {best_score:.3f} = "
            f"{a:.1f}*{norm_obstacle[best_index]:.3f}(障碍) + "
            f"{b:.1f}*{norm_angle[best_index]:.3f}(角度) + "
            f"{c:.1f}*{norm_radius:.3f}(半径) + "
            f"{d:.1f}*{raw_distance_scores[best_index]:.3f}(距离)"  # 新增距离评分输出
        )

    
    def calculate_obstacle_score(self, candidate, obstacles):
        """计算到最近障碍物的距离"""
        if not obstacles:
            self.get_logger().debug("无障碍物数据，使用默认距离值10.0")
            return 10.0  # 无障碍物时返回较大值
            
        min_dist = float('inf')
        for obs in obstacles:
            dist = self.calculate_distance(candidate, obs)
            if dist < min_dist:
                min_dist = dist
                
        self.get_logger().debug(f"点({candidate.x:.2f},{candidate.y:.2f})到最近障碍物距离: {min_dist:.2f}")
        return min_dist
    
    def calculate_angle_score(self, candidate, center, refer):
        """计算与参考向量的角度差（0-90度）"""
        vec_ref = [refer.x - center.x, refer.y - center.y]
        vec_cand = [candidate.x - center.x, candidate.y - center.y]
        
        # 计算点积
        dot_product = vec_ref[0]*vec_cand[0] + vec_ref[1]*vec_cand[1]
        
        # 计算模长
        norm_ref = math.sqrt(vec_ref[0]**2 + vec_ref[1]**2)
        norm_cand = math.sqrt(vec_cand[0]**2 + vec_cand[1]**2)
        
        if norm_ref < 1e-6 or norm_cand < 1e-6:
            return 90.0  # 处理零向量情况
            
        # 计算夹角余弦值（限制在[-1,1]范围内）
        cos_theta = dot_product / (norm_ref * norm_cand)
        cos_theta = max(-1.0, min(1.0, cos_theta))
        
        # 计算角度（度）
        angle_rad = math.acos(cos_theta)
        angle_deg = math.degrees(angle_rad)
        
        # 取锐角（0-90度）
        acute_angle = min(angle_deg, 180 - angle_deg)
        
        return acute_angle
    
    def calculate_distance(self, p1, p2):
        """计算两点间距离"""
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dist = math.sqrt(dx**2 + dy**2)
        return dist
    
    def normalize_scores(self, scores, higher_better=True):
        """归一化得分到[0,1]区间"""
        if not scores:
            self.get_logger().warn("归一化空分数列表，返回空列表")
            return []
            
        min_score = min(scores)
        max_score = max(scores)
        score_range = max_score - min_score
        
        if score_range < 1e-6:  # 防止除零
            self.get_logger().warn("分数范围过小，使用默认归一化值0.5")
            return [0.5] * len(scores)
            
        if higher_better:
            normalized = [(s - min_score) / score_range for s in scores]
        else:
            # 对于距离得分，距离越小得分越高
            normalized = [(max_score - s) / score_range for s in scores]
        
        return normalized
    
    def normalize_robot_distance(self, distances, valid_position, decay_factor=1.5):
        """
        改进的距离归一化方法（指数衰减）
        :param distances: 原始距离列表
        :param valid_position: 机器人位置是否有效
        :param decay_factor: 距离衰减因子（值越大近距离得分越高）
        :return: 归一化后的得分列表
        """
        if not valid_position or not distances:
            # 位置无效时返回全零列表
            self.get_logger().warn("⚠️ 机器人位置无效，距离评分设为0")
            return [0.0] * len(distances)
        
        # 1. 计算基础归一化得分
        min_dist = min(distances)
        max_dist = max(distances)
        dist_range = max_dist - min_dist
        
        # 处理微小距离差（避免除零）
        if dist_range < 0.1:  # 当距离差<10cm时视为相同距离
            self.get_logger().info("距离差异小于10cm，使用统一中间值0.5")
            return [0.5] * len(distances)
        
        # 基础线性归一化（距离越小得分越高）
        base_scores = [(max_dist - d) / dist_range for d in distances]
        
        # 2. 引入距离衰减因子（指数衰减更符合导航需求）
        decayed_scores = [min(1.0, score ** (1/decay_factor)) for score in base_scores]
        
        # 打印距离信息
        self.get_logger().info(
            f"机器人位置: ({self.current_robot_position.x:.2f}, {self.current_robot_position.y:.2f}) | "
            f"距离范围: {min_dist:.2f}-{max_dist:.2f}m | "
            f"衰减因子: {decay_factor:.2f}"
        )
        
        return decayed_scores

def main(args=None):
    rclpy.init(args=args)
    node = OptimalPointSelector()
    
    try:
        node.get_logger().info("节点启动，开始处理数据...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断操作，节点关闭中...")
    except Exception as e:
        node.get_logger().error(f"节点运行异常: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("节点已安全关闭")

if __name__ == '__main__':
    main()