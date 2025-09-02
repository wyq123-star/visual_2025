#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math
import time
from rcl_interfaces.msg import IntegerRange
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType

class RandomPointGenerator(Node):
    """独立的地图处理和随机点生成模块"""
    def __init__(self): 
        super().__init__('random_point_generator')
        self.candidate_points = []  # 存储候选点
        self.base_angle = 0.0  # 半圆环基准角度
        self.publish_counter = 0  # 发布计数器
        self.last_generation_time = 0.0  # 上次生成时间
        
        # 声明动态参数
        self.declare_parameter('map_x', 7.0)  # 地图X尺寸
        self.declare_parameter('map_y', -14.0)  # 地图Y尺寸
        self.declare_parameter('origin_x', 0.0)  # 原点X坐标
        self.declare_parameter('origin_y', 0.0)  # 原点Y坐标
        self.declare_parameter('center_x', 3.5)  # 圆心X坐标
        self.declare_parameter('center_y', -14.0)  # 圆心Y坐标
        self.declare_parameter('radius_min', 3.0)  # 最小半径
        self.declare_parameter('radius_max', 4.0)  # 最大半径
        self.declare_parameter('num_points', 36)  # 候选点数量
        
        # 新增：安全距离参数（默认0.5米）
        self.declare_parameter(
            'safety_distance', 0.5,
            ParameterDescriptor(
                description='点与矩形边界的安全距离',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        # 新增：发布模式参数 (fixed/dynamic)
        self.declare_parameter(
            'publish_mode', 'fixed',
            ParameterDescriptor(
                description='候选点发布模式: fixed=固定一组点, dynamic=持续生成新点',
                type=ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints="Allowed values: ['fixed', 'dynamic']"
            )
        )
        
        # 新增：持续发布参数
        self.declare_parameter(
            'continuous_publish', True,
            ParameterDescriptor(
                description='是否持续发布候选点',
                type=ParameterType.PARAMETER_BOOL
            )
        )
        
        # 新增：发布频率参数 (Hz) - 提高到2.0Hz
        self.declare_parameter(
            'publish_frequency', 2.0,  # 从1.0提高到2.0Hz
            ParameterDescriptor(
                description='候选点发布频率 (Hz)',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        # 新增：半圆环模式参数
        self.declare_parameter(
            'half_circle', True,
            ParameterDescriptor(
                description='是否只生成半圆环内的点',
                type=ParameterType.PARAMETER_BOOL
            )
        )
        
        # 新增：点生成稳定性参数
        self.declare_parameter(
            'min_generation_interval', 0.5,  # 最小生成间隔（秒）
            ParameterDescriptor(
                description='点生成的最小时间间隔',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        # 坐标精度参数：配合电控
        self.declare_parameter(
            'coordinate_precision', 'high',
            ParameterDescriptor(
                description='坐标精度: high=高精度浮点, low=低精度小数点后一位',
                type=ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints="Allowed values: ['high', 'low']"
            )
        )
        
        # 新增：小数点位数参数 (0-5位)
        self.declare_parameter(
            'decimal_places', 1,
            ParameterDescriptor(
                description='低精度模式下的小数点位数 (0=整数精度, 1-5=小数点后位数)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(  # 使用IntegerRange对象
                    from_value=0,
                    to_value=5,
                    step=1
                )]
            )
        )
        
        # 创建候选点发布器
        self.points_pub = self.create_publisher(
            PoseArray,
            '/points_select',
            10
        )
        
        # 添加参数回调
        self.add_on_set_parameters_callback(self.param_callback_handler)
        
        # 计算基准角度（圆心到地图原点的方向）
        self.calculate_base_angle()
        
        # 生成初始候选点
        self.generate_candidate_points()
        
        # 创建定时器用于持续发布
        self.create_publish_timer()
        
        # 创建点生成定时器（用于动态模式）
        self.create_generation_timer()
        
        self.get_logger().info("随机点生成节点已启动")
    
    def create_publish_timer(self):
        """创建或更新发布定时器"""
        # 如果已有定时器，先取消
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
        
        # 获取发布频率参数
        frequency = self.get_parameter('publish_frequency').value
        if frequency <= 0:
            frequency = 2.0  # 默认2Hz
        
        # 创建新定时器
        self.publish_timer = self.create_timer(
            1.0 / frequency,  # 秒
            self.publish_candidate_points
        )
    
    def create_generation_timer(self):
        """创建或更新点生成定时器（用于动态模式）"""
        # 如果已有定时器，先取消
        if hasattr(self, 'generation_timer'):
            self.generation_timer.cancel()
        
        # 获取发布模式
        publish_mode = self.get_parameter('publish_mode').value
        
        # 仅在动态模式下创建生成定时器
        if publish_mode == 'dynamic':
            min_interval = self.get_parameter('min_generation_interval').value
            if min_interval <= 0:
                min_interval = 0.5  # 默认0.5秒
            
            # 创建新定时器
            self.generation_timer = self.create_timer(
                min_interval,  # 秒
                self.generate_candidate_points
            )
    
    def param_callback_handler(self, params):
        """处理参数更新"""
        for param in params:
            param_name = param.name
            # 当关键参数变化时重新生成点
            if param_name in ['center_x', 'center_y', 'radius_min', 'radius_max', 
                             'num_points', 'publish_mode', 'origin_x', 'origin_y', 
                             'half_circle', 'safety_distance', 'map_x', 'map_y',
                             'coordinate_precision', 'decimal_places']:
                self.get_logger().info(
                    f"参数更新: {param_name} = {param.value}"
                )
                self.generate_candidate_points()
            
            # 当发布频率变化时更新定时器
            elif param_name == 'publish_frequency':
                self.create_publish_timer()
            
            # 当持续发布设置变化时
            elif param_name == 'continuous_publish':
                if param.value:
                    self.create_publish_timer()
                elif hasattr(self, 'publish_timer'):
                    self.publish_timer.cancel()
            
            # 当点生成间隔或模式变化时更新生成定时器
            elif param_name in ['min_generation_interval', 'publish_mode']:
                self.create_generation_timer()
        
        return SetParametersResult(successful=True)
    
    def calculate_base_angle(self):
        """计算半圆环基准角度（圆心到地图原点的方向）"""
        origin_x = self.get_parameter('origin_x').value
        origin_y = self.get_parameter('origin_y').value
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        
        # 计算从圆心指向地图原点的向量
        dx = origin_x - center_x
        dy = origin_y - center_y
        
        # 使用atan2计算角度（弧度）
        self.base_angle = math.atan2(dy, dx)
        self.get_logger().info(f"半圆环基准角度: {math.degrees(self.base_angle):.2f}°")
    
    def is_point_in_safe_rectangle(self, x, y):
        """检查点是否在安全矩形区域内"""
        # 获取参数值
        origin_x = self.get_parameter('origin_x').value
        origin_y = self.get_parameter('origin_y').value
        map_x = self.get_parameter('map_x').value
        map_y = self.get_parameter('map_y').value
        safety_distance = self.get_parameter('safety_distance').value
        
        # 计算矩形边界（考虑安全距离）
        rect_min_x = min(origin_x, origin_x + map_x) + safety_distance
        rect_max_x = max(origin_x, origin_x + map_x) - safety_distance
        rect_min_y = min(origin_y, origin_y + map_y) + safety_distance
        rect_max_y = max(origin_y, origin_y + map_y) - safety_distance
        
        # 检查点是否在安全矩形内
        return (rect_min_x <= x <= rect_max_x and 
                rect_min_y <= y <= rect_max_y)

    def generate_candidate_points(self):
        """在圆环区域内生成均匀分布的候选点（可选择半圆环）"""
        # 获取坐标精度模式
        precision_mode = self.get_parameter('coordinate_precision').value
        
        # 根据精度模式选择生成方法
        if precision_mode == 'high':
            self.generate_high_precision_points()
        else:  # 低精度模式
            self.generate_low_precision_points()
    
    def generate_high_precision_points(self):
        """高精度浮点生成方法"""
        # 获取动态参数值
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        radius_min = self.get_parameter('radius_min').value
        radius_max = self.get_parameter('radius_max').value
        num_points = self.get_parameter('num_points').value
        half_circle = self.get_parameter('half_circle').value
        safety_distance = self.get_parameter('safety_distance').value
        
        # 清空候选点列表
        self.candidate_points = []
        
        # 计算角度范围（半圆环或全圆环）
        if half_circle:
            # 半圆环模式：180度范围（π弧度）
            start_angle = self.base_angle - math.pi/2
            end_angle = self.base_angle + math.pi/2
            angle_range = math.pi
        else:
            # 全圆环模式：360度范围（2π弧度）
            start_angle = 0
            end_angle = 2 * math.pi
            angle_range = 2 * math.pi
        
        # 在圆环区域内均匀生成点
        valid_points = 0
        max_attempts = num_points * 5  # 最大尝试次数
        
        for attempt in range(max_attempts):
            if valid_points >= num_points:
                break
                
            # 计算角度（均匀分布）
            angle = start_angle + (valid_points / num_points) * angle_range
            
            # 在半径范围内随机选择半径
            radius = random.uniform(radius_min, radius_max)
            
            # 计算点的坐标（高精度浮点）
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            # 检查点是否在安全矩形内
            if self.is_point_in_safe_rectangle(x, y):
                self.candidate_points.append((x, y))
                valid_points += 1
        
        # 如果有效点不足，记录警告
        if len(self.candidate_points) < num_points:
            self.get_logger().warn(
                f"高精度模式仅生成 {len(self.candidate_points)}/{num_points} 个有效点（安全距离: {safety_distance}m）"
            )
    
    def generate_low_precision_points(self):
        """低精度生成方法（动态小数点位数控制）"""
        # 获取动态参数值
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        radius_min = self.get_parameter('radius_min').value
        radius_max = self.get_parameter('radius_max').value
        num_points = self.get_parameter('num_points').value
        half_circle = self.get_parameter('half_circle').value
        safety_distance = self.get_parameter('safety_distance').value
        decimal_places = self.get_parameter('decimal_places').value  # 获取小数点位数参数
        
        # 计算精度步长（基于小数点位数）
        precision_step = 10 ** (-decimal_places)  # 如decimal_places=2 -> 0.01
        
        # 清空候选点列表
        self.candidate_points = []
        
        # 计算角度范围（半圆环或全圆环）
        if half_circle:
            # 半圆环模式：180度范围（π弧度）
            start_angle = self.base_angle - math.pi/2
            end_angle = self.base_angle + math.pi/2
            angle_range = math.pi
        else:
            # 全圆环模式：360度范围（2π弧度）
            start_angle = 0
            end_angle = 2 * math.pi
            angle_range = 2 * math.pi
        
        # 在圆环区域内均匀生成点
        valid_points = 0
        max_attempts = num_points * 5  # 最大尝试次数
        
        for attempt in range(max_attempts):
            if valid_points >= num_points:
                break
                
            # 计算角度（均匀分布）
            angle = start_angle + (valid_points / num_points) * angle_range
            
            # 在半径范围内生成指定精度的半径
            # 计算可用的离散半径数量
            radius_steps = int((radius_max - radius_min) / precision_step) + 1
            step_idx = random.randint(0, radius_steps - 1)
            radius = round(radius_min + step_idx * precision_step, decimal_places)
            
            # 计算点的坐标（应用指定精度）
            x = round(center_x + radius * math.cos(angle), decimal_places)
            y = round(center_y + radius * math.sin(angle), decimal_places)
            
            # 检查点是否在安全矩形内
            if self.is_point_in_safe_rectangle(x, y):
                self.candidate_points.append((x, y))
                valid_points += 1
        
        # 如果有效点不足，记录警告
        if len(self.candidate_points) < num_points:
            self.get_logger().warn(
                f"低精度模式仅生成 {len(self.candidate_points)}/{num_points} 个有效点"
                f"（精度: 小数点后{decimal_places}位, 安全距离: {safety_distance}m）"
            )
    
    def publish_candidate_points(self):
        """发布候选点到/points_select话题"""
        if not self.candidate_points:
            return
            
        pose_array = PoseArray()
        pose_array.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="map"
        )
        
        # 获取当前精度模式用于日志
        precision_mode = self.get_parameter('coordinate_precision').value
        precision_info = "高精度" if precision_mode == 'high' else "低精度"
        
        # 获取小数点位数
        decimal_places = self.get_parameter('decimal_places').value
        
        for point in self.candidate_points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # 无旋转
            pose_array.poses.append(pose)
        
        self.points_pub.publish(pose_array)
        self.publish_counter += 1
        
        # 获取当前发布模式
        publish_mode = self.get_parameter('publish_mode').value
        mode_info = "固定" if publish_mode == 'fixed' else "动态"
        
        # 获取半圆环模式
        half_circle = self.get_parameter('half_circle').value
        circle_info = "半圆环" if half_circle else "全圆环"
        
        # 获取安全距离
        safety_distance = self.get_parameter('safety_distance').value
        
        # 定期记录日志（每10次发布记录一次）
        if self.publish_counter % 10 == 0:
            precision_detail = ""
            if precision_mode == 'low':
                precision_detail = f"(小数点后{decimal_places}位)"
            
            self.get_logger().info(
                f"已发布 {self.publish_counter} 次候选点 "
                f"({mode_info}模式, {circle_info}, {precision_info}{precision_detail}, "
                f"安全距离: {safety_distance}m)"
            )

def main(args=None):
    rclpy.init(args=args)
    node = RandomPointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()