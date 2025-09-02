#!/usr/bin/env python3
import rclpy
import time,json
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from pid import pid_increase_t
from tf2_ros import TransformListener, Buffer,LookupTransform
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
import math
from geometry_msgs.msg import Twist
from rclpy.time import Time
from std_msgs.msg import String
class EnhancedNavigationHandler:
    """支持动态目标点跟踪和参数动态调整"""
    IDLE = 0          # 空闲状态，等待新目标
    NAVIGATING = 1    # 导航中状态
    YAW =2       # 只对齐yaw角状态
    YAW_ONLY=3  # 调试用
    def __init__(self, node:Node):
        self.node = node
        self.current_state = self.IDLE #状态
        self.current_goal_handle = None
        self.last_goal_time = 0.0
        self.failure_count = 0
        self.max_failures = 20  # 最大失败次数提高到20次
        self.active_goal:Point= None  # 当前活跃目标点
        self.best_goal:Point = None  # 最佳目标点
        self.node.declare_parameter("pid_distance",0.2) #进入pid对齐的距离阈值
        self.node.declare_parameter("map_frame", "map")  # 地图坐标系ID
        self.node.declare_parameter("base_link_frame","base_link")  # 基座坐标系ID
        self.node.declare_parameter('center_x', 14.0)
        self.node.declare_parameter('center_y', 3.5)
        self.pid_distance = self.node.get_parameter("pid_distance").value
        self.map_frame = self.node.get_parameter("map_frame").value
        self.base_link_frame = self.node.get_parameter("base_link_frame").value
        self.center_x = self.node.get_parameter('center_x').value
        self.center_y = self.node.get_parameter('center_y').value
        self.pid_x=pid_increase_t(0.3,1,0.2, -0.5, 0.5)  # PID控制器参数
        self.pid_y=pid_increase_t(0.3,1,0.2, -0.5, 0.5)  
        self.pid_yaw=pid_increase_t(0.6,1.4,0.4, -2.0, 2.0)
        self.min_velocity = 0.1  # 最小速度阈值
        self.min_yaw_velocity = 0.2 # 最小yaw速度阈值
        self.local_threshold=0.015 #pid对齐的局部阈值
        self.yaw_threshold = 0.013  # yaw对齐的阈值
        self.yaw_finsih_threshold=0.02 # yaw对齐完成的阈值
        # 创建Action客户端连接官方导航
        self.active_goal = None
        #导航标志位
        self.nav_success_flag=False
        self.nav_reset=False
        self.handle_reset = False # 手动重新导航 
        self.align_finished=False # 对齐完成标志

        # 声明动态参数（带默认值）
        self.node.declare_parameter('max_failures', 20)
        self.node.declare_parameter('goal_timeout', 60.0)
        self.node.declare_parameter('is_dynamic', True)  # 动态打断参数
        
        # 注册参数回调
        self.node.add_on_set_parameters_callback(self.parameters_callback)
        
        # 初始化参数值
        self.max_failures = self.node.get_parameter('max_failures').value
        self.goal_timeout = self.node.get_parameter('goal_timeout').value
        self.is_dynamic = self.node.get_parameter('is_dynamic').value  # 是否允许动态打断
        
        # 创建Action客户端
        self.nav_client = ActionClient(
            self.node, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self.node)
        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # 发布导航目标
        self.goal_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile
        )
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        # 订阅优化点话题
        self.optimal_sub = self.node.create_subscription(
            Point,
            '/optimal_point_data',
            self.set_goal,
            10
        )
        #订阅车体状态话题
        self.state_sub = self.node.create_subscription(String,
            '/robot_state',
            self.state_callback,
            10
        )
        self.state_pub= self.node.create_publisher(
            String,
            '/robot_state',
            qos_profile
        )
        self.state_timer = self.node.create_timer(0.02, self.state_update)
        self.republish_timer = self.node.create_timer(1, self.republish_goal)
        self.node.get_logger().info(
            f"🚀 导航处理器初始化完成 | max_failures={self.max_failures} | goal_timeout={self.goal_timeout}s | is_dynamic={self.is_dynamic}"
        )
        if self.current_state==self.YAW_ONLY:
            self.goal_sub= self.node.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.goal_sub, #回调函数
                10
            )
    def parameters_callback(self, params):
        """处理参数变化的回调函数"""
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'max_failures':
                self.max_failures = param.value # 更新最大失败次数
                self.node.get_logger().info(f"📌 更新 max_failures = {self.max_failures}")
            elif param.name == 'goal_timeout':
                self.goal_timeout = param.value
                self.node.get_logger().info(f"⏱️ 更新 goal_timeout = {self.goal_timeout}s")
            # 新增：动态打断参数处理
            elif param.name == 'is_dynamic':
                self.is_dynamic = param.value
                self.node.get_logger().info(f"🌀 更新 is_dynamic = {self.is_dynamic}")
                # 参数切换时清空等待中的目标
                if not self.is_dynamic and self.pending_goal:
                    self.node.get_logger().info("🛑 关闭动态模式，清空等待目标")
                    self.pending_goal = None
        return result


    def set_goal(self, point):
        self.best_goal = point

    
    def publish_goal(self, point):
        """发布导航目标"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = point.x
        goal_msg.pose.position.y = point.y
        goal_msg.pose.position.z = 0.0
        #从目标点和篮筐中心点算出来目标yaw角
        
        target_yaw= self.normalize_angle(math.atan2(self.center_y-point.y, self.center_x-point.x))
        #仿真里面旋转了90度
        # else :
            # target_yaw =self.normalize_angle( math.atan2(point.y - self.center_y, point.x - self.center_x) + math.pi)
        #从yaw 算出来四元数
        z= math.sin(target_yaw / 2.0)
        w= math.cos(target_yaw / 2.0)
        goal_msg.pose.orientation.z = z
        goal_msg.pose.orientation.w = w
        
        self.goal_publisher.publish(goal_msg)
        self.node.get_logger().info(f"📍 发布目标: x={point.x:.2f}, y={point.y:.2f}")
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("🚨 导航服务器连接超时")
            # self.reset_state()
            return
        
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal, 
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    def nav_feedback_callback(self, feedback):
        """处理导航反馈"""
        """_summary_ 暂时不做处理
        """
    def nav_result_callback(self, future:rclpy.Future):
        """导航是否成功的回调"""
        result= future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_success_flag = True
        else:
            self.nav_reset = True
    def goal_response_callback(self, future:rclpy.Future):
        """是否接受导航目标的回调"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error("🚨 导航目标被拒绝")
                # self.reset_state()
                self.nav_reset = True
                return
            self.node.get_logger().info("✅ 导航目标已接受，开始导航")
            
            # 设置导航结果回调
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)

        except Exception as e:
            self.node.get_logger().error(f"🚨 导航目标响应异常: {str(e)}")
            # self.reset_state()
    def pid_align(self,point:Point):
        # if self.active_align is False:
            # return
        if point is None:
            return
        current_pose=self.buffer.lookup_transform(
            self.map_frame, 
            self.base_link_frame,time=Time())
        current_yaw=self.normalize_angle( math.atan2(
            current_pose.transform.rotation.z, 
            current_pose.transform.rotation.w) * 2.0)
        

        # target_yaw = self.normalize_angle(math.atan2(
        #     self.center_y - point.y, 
        #     self.center_x - point.x))
        #计算当前的yaw 角
        current_x= current_pose.transform.translation.x
        current_y= current_pose.transform.translation.y
        target_yaw=self.normalize_angle(math.atan2(
            self.center_y - current_y,
            self.center_x - current_x))
        
        #yaw 有过零点检测问题
        # error_yaw=self.normalize_angle(target_yaw - current_yaw)
        self.pid_x.set_target(point.x)
        self.pid_y.set_target(point.y)
        self.pid_yaw.set_target(target_yaw)
        # error_yaw = target_yaw - current_yaw
        # PID控制器计算
        #控制指令
        control_x = self.pid_x.update(current_pose.transform.translation.x)
        control_y = self.pid_y.update(current_pose.transform.translation.y)
        control_yaw = self.pid_yaw.update(current_yaw)
        control_yaw= self.normalize_angle(control_yaw)
        #将x y 转移到全局坐标系
        if abs(control_x) <self.min_velocity:
            control_x=abs(control_x) * self.min_velocity / control_x
        if abs(control_y) <self.min_velocity:
            control_y=abs(control_y) * self.min_velocity / control_y
        if abs(control_yaw) < self.min_yaw_velocity:
            control_yaw = abs(control_yaw) * self.min_yaw_velocity / control_yaw
        #判断是否对齐
        if abs(self.pid_x.error_last) < self.local_threshold :
            control_x = 0.0
        if abs(self.pid_y.error_last) < self.local_threshold :
            control_y = 0.0
        if abs(self.pid_yaw.error_last) < self.yaw_threshold :
            control_yaw = 0.0
        control_x_local = control_x * math.cos(current_yaw) - control_y * math.sin(current_yaw)
        # control_y = control_x * math.sin(current_yaw) + control_y * math.cos(current_yaw)
        control_y_local = control_y * math.cos(current_yaw) - control_x * math.sin(current_yaw)
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = control_yaw
        # 发布速度指令
        self.cmd_vel_publisher.publish(cmd_vel)
        if abs(self.pid_yaw.error_last) < self.yaw_finsih_threshold and self.align_finished is False:
            self.align_finished = True
            self.state_pub.publish(String(data=json.dumps({'nav_state': 'ALIGNED'})))
    def state_update(self):
        # print("state is {}".format(self.current_state))
        if self.current_state==self.IDLE:
            if self.best_goal is None:
                return
            #发布目标点
            self.active_goal= self.best_goal
            print("\033[1;35m point x:{} y:{}\033[0m".format(self.best_goal.x,self.best_goal.y))
            self.publish_goal(self.active_goal)

            #切换状态
            self.current_state = self.NAVIGATING
            self.align_finished= False
        elif self.current_state == self.NAVIGATING:
            if self.nav_success_flag:
                self.current_state = self.YAW
                self.nav_success_flag = False
            if self.nav_reset:
                self.current_state = self.IDLE
                self.nav_reset = False
        elif self.current_state == self.YAW:
            self.nav_reset= False
            self.pid_align(point=self.active_goal)
            if self.handle_reset:
                self.current_state = self.IDLE
                self.handle_reset = False
        elif self.current_state == self.YAW_ONLY:
            # 只对齐yaw角
            self.pid_align(point=self.active_goal)
    def republish_goal(self):
        self.nav_reset=True
    def normalize_angle(self,angle):
        """把任意弧度归一化到 [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    def goal_sub(self,Pose:PoseStamped):
        """订阅目标点"""
        self.active_goal= Point()
        self.active_goal.x= Pose.pose.position.x
        self.active_goal.y= Pose.pose.position.y
    def state_callback(self, msg: String):
        data= json.loads(msg.data)
        #检测nav_state是否存在
        if 'nav_state' in data:
            nav_state = data['nav_state']
            if nav_state == "IDLE":
                self.handle_reset = True
            


class OptimalGoalNavigator(Node):
    """最优目标导航节点"""
    def __init__(self):
        super().__init__('optimal_goal_navigator')
        self.navigation_handler = EnhancedNavigationHandler(self)
        self.get_logger().info("🚀 最优目标导航节点已启动")

def main(args=None):
    rclpy.init(args=args)
    node = OptimalGoalNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被手动终止")
        node.navigation_handler.cancel_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()