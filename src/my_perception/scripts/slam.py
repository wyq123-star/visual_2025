#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import json, os, math, numpy as np
import rclpy.time
from std_msgs.msg import String, Float32  # 新增Float32导入
from itertools import product
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster

class fusion_node_t(Node):
    def __init__(self):
        super().__init__('fusion_node')
        # 原有参数声明
        self.declare_parameter('odom_frame','odom_wheel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('slam_odom',['camera_init'])
        self.declare_parameter('slam_base_link',['body','aft_mapped'])
        self.declare_parameter('odom_topic','/odom')
        self.declare_parameter('base_to_laser', [0.085,0.095, 0.0])
        self.declare_parameter('riqiang_y', -0.10975)
        self.declare_parameter('slam_to_map',[0.46876+0.26775,-0.08475-0.0815,0.0])
        
        # 新增Sick相关参数
        self.declare_parameter('sick_topic', '/sick')          # Sick话题名称
        self.declare_parameter('sick_buffer_size', 10)         # Sick数据缓存大小
        self.declare_parameter('sick_lateral_offset', 0.0)     # Sick到base_link的横向偏移补偿
        
        # 获取参数值
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.slam_to_map = self.get_parameter('slam_to_map').value
        self.base_to_laser = self.get_parameter('base_to_laser').value
        self.slam_odom = self.get_parameter('slam_odom').value
        self.slam_base_link = self.get_parameter('slam_base_link').value
        
        self.sick_topic = self.get_parameter('sick_topic').value
        self.sick_buffer_size = self.get_parameter('sick_buffer_size').value
        self.sick_lateral_offset = self.get_parameter('sick_lateral_offset').value
        
        self.sick_buffer = []  # 存储Sick横向位移数据
        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #创建均值滤波器 主要用于slam_tf_callback(self)函数
        self.tf_overage_x = []
        self.tf_overage_y = []
        self.tf_overage_w=[]
        self.tf_overage_z=[]
        self.tf_overage_yaw=np.array([], dtype=np.float32)  # 用于存储yaw的均值滤波

        #创建slam数据缓存
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_yaw = 0.0

        self.odom_x=0.0
        self.odom_y=0.0
        self.odom_yaw=0.0
        self.base_link_x=0.0
        self.base_link_y=0.0
        self.r = math.sqrt(self.base_to_laser[0]**2 + self.base_to_laser[1]**2)
        self.laser_angle = math.atan2(self.base_to_laser[1], self.base_to_laser[0])

        #两个定时器回调和两个订阅者回调
        self.slam_timer = self.create_timer(0.005,self.slam_tf_callback)
        self.improved_slam_timer = self.create_timer(0.01, self.publish_improved_slam)
        self.slam_odom_pub = self.create_publisher(Odometry,'slam_improved',10)
        self.improved_slam_sub  = self.create_subscription(Odometry,'/slam_improved',self.improved_slam_callback,10)
        self.odom_sub = self.create_subscription(Vector3Stamped,self.odom_topic,self.fuse_callback,10)
        self.odom_pub= self.create_publisher(Vector3Stamped, 'base_link_odom', 10)
        self.robot_sub= self.create_subscription(String, 'robot_state', self.robot_state_callback, 1)
        
        self.sick_sub = self.create_subscription(
            Float32,  # 假设/sick发布的是Float32类型的横向位移值；如果是Vector3Stamped需修改
            self.sick_topic,
            self.sick_callback,
            10
        )

    def sick_callback(self, msg: Float32):
        """接收Sick雷达数据，存入缓存区（保持固定大小）"""
        # 如果是Vector3Stamped类型（比如横向位移在y分量），修改为：
        # self.sick_buffer.append(msg.vector.y + self.sick_lateral_offset)
        self.sick_buffer.append(msg.data + self.sick_lateral_offset)
        
        # 限制缓存大小，超出则移除最旧数据
        if len(self.sick_buffer) > self.sick_buffer_size:
            self.sick_buffer.pop(0)


    def slam_tf_callback(self):
        # print(f'{self.r},{self.laser_angle}')
        transform=TransformStamped()
        if len(self.slam_odom) >0 and len(self.slam_base_link) > 0:
            #尝试找出其中能用的tf
            for map_frame, base_frame in product(self.slam_odom, self.slam_base_link): #遍历所有frame组合
                try:
                    if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                        # self.get_logger().warn(f"Transform from {map_frame} to {base_frame} not available")
                        continue
                    transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
                    # self.get_logger().info(f"Transform from {map_frame} to {base_frame} not available")
                    break  # 找到一个可用的就退出循环
                except Exception as e:
                    self.get_logger().error(f"Failed to get transform from {map_frame} to {base_frame}: {e}")
        else:
            try:
                if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                    # self.get_logger().warn(f"Transform from {map_frame} to {base_frame} not available")
                    return
                transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"Failed to get transform: {e}")
                return
        # self.tf_overage_buffer.append(transform)
        self.tf_overage_x.append(transform.transform.translation.x)
        self.tf_overage_y.append(transform.transform.translation.y)
        self.tf_overage_w.append(transform.transform.rotation.w)
        self.tf_overage_z.append(transform.transform.rotation.z)
        yaw= 2 * math.atan2(transform.transform.rotation.z, transform.transform.rotation.w)
        self.tf_overage_yaw = np.append(self.tf_overage_yaw, yaw)

    def publish_improved_slam(self):
        """功能描述:这个函数是一个定时器函数,频率为100hz,把在slam_tf_callback()函数里存储下来的x,y,z,w,yaw进行一个均值滤波之后通过一个话题发布(100hz发布)"""
        """
            参数说明：
            x_list 之前tf的x数据是缓存了，所以这个变量是把缓存的x数据copy过来，其余类似参数同理，这个变量只是局部变量，所以不需要手动清除
            laser_odom_x 均值滤波之后的x
        """
        # print('1')
        if len(self.tf_overage_x) == 0:
            return
            
        # 创建数据的本地副本，以便在锁外进行计算
        x_list = self.tf_overage_x.copy()
        # print(f'{x_list}')
        y_list = self.tf_overage_y.copy()
        w_list = self.tf_overage_w.copy()
        z_list = self.tf_overage_z.copy()
        yaw_list = self.tf_overage_yaw.copy()
            
        # 清空缓存，为下一轮数据采集做准备
        self.tf_overage_x.clear()
        self.tf_overage_y.clear()
        self.tf_overage_w.clear()
        self.tf_overage_z.clear()
        self.tf_overage_yaw = np.array([], dtype=np.float32)
        
        laser_odom_x = sum(x_list) / len(x_list)
        # print(f'当前laser_odom_x列表: {laser_odom_x}')
        laser_odom_y = sum(y_list) / len(y_list)
        w = sum(w_list) / len(w_list)
        z = sum(z_list) / len(z_list)
        
        # 计算角度均值（正确处理角度环绕）
        # 将yaw转换为四元数
        sin_sum = np.sum(np.sin(yaw_list))
        cos_sum = np.sum(np.cos(yaw_list))
        mean_yaw = math.atan2(sin_sum, cos_sum)  # 计算均值yaw
        
        # ==================== Odometry消息发布（原有逻辑） ====================
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "camera_init_improved"  # 父坐标系
        odom_msg.child_frame_id = "aft_mapped_improved"   # 子坐标系
        
        odom_msg.pose.pose.position.x = laser_odom_x 
        odom_msg.pose.pose.position.y = laser_odom_y 
        odom_msg.pose.pose.position.z = 0.0
        
        # 将yaw转换为四元数
        qw = math.cos(mean_yaw / 2.0)
        qz = math.sin(mean_yaw / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # 设置协方差
        odom_msg.pose.covariance = [0.1] * 36
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [0.1] * 36
        
        self.slam_odom_pub.publish(odom_msg)

    def improved_slam_callback(self,msg:Odometry):
        """功能描述：这是一个ros2订阅者的回调函数，一旦有提升过频率的slam话题发布，就会记录相应的数据，为之后的数据融合做准备"""
        """
            参数声明：
            self.slam_x 用于记录最新一次话题的数据，其他同理
        """
        
        try:
            # 原始坐标
            original_x = msg.pose.pose.position.x
            original_y = msg.pose.pose.position.y
            
            # 根据旋转角度处理坐标变换
            if self.base_to_laser[2] == 0:
                # 0度：无旋转
                self.slam_x = original_x
                self.slam_y = original_y
            elif self.base_to_laser[2] == 90:
                # 90度：x' = -y, y' = x
                self.slam_x = -original_y
                self.slam_y = original_x
            elif self.base_to_laser[2] == 180:
                # 180度：x' = -x, y' = -y
                self.slam_x = -original_x
                self.slam_y = -original_y
            elif self.base_to_laser[2] == 270:
                # 270度：x' = y, y' = -x
                self.slam_x = original_y
                self.slam_y = -original_x
            else:
                # 默认情况（理论上不会发生，因为参数限制在四种值）
                self.slam_x = original_x
                self.slam_y = original_y
                self.get_logger().warning(f"不支持的旋转角度: {self.base_to_laser[2]}，使用默认0度处理")
            
            # 处理姿态（四元数转偏航角）
            orientation = msg.pose.pose.orientation
            siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            original_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # 应用旋转角度到偏航角（将角度转换为弧度）
            rotation_rad = math.radians(self.base_to_laser[2])
            self.slam_yaw = original_yaw
            
            # 规范化偏航角到[-π, π]范围
            if self.slam_yaw > math.pi:
                self.slam_yaw -= 2 * math.pi
            elif self.slam_yaw < -math.pi:
                self.slam_yaw += 2 * math.pi
                
            self.slam_stamp = msg.header.stamp
            
        except Exception as e:
            self.get_logger().error(f"SLAM回调处理错误: {e}")


    def fuse_callback(self,msg:Vector3Stamped):
        """功能描述：这是ros2的/odom话题订阅者的回调函数，一旦有odom话题发布，就会获取记录的slam的数据，并进行数据融合，把轮式里程计的偏差纠正后，通过话题发布车体坐标"""
        """
            参数声明：
            self.odom_x 有话题就记录下来数据，其余同理
            base_link_tf 获取map到车体中心的tf的偏移
        """
        x_diff,y_diff,yaw_diff = 0.0,0.0,0.0
        self.odom_x = msg.vector.x
        self.odom_y = msg.vector.y
        self.odom_yaw = msg.vector.z
        self.tf_publish(self.odom_frame, self.base_frame, self.odom_x, self.odom_y, self.odom_yaw)
        dyaw= self.slam_yaw - self.odom_yaw
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi

        self.base_link_x=self.slam_x - self.r*math.sin(self.laser_angle + self.slam_yaw) +self.base_to_laser[1]
        self.base_link_y=self.slam_y + self.r*math.cos(self.laser_angle + self.slam_yaw) -self.base_to_laser[0]
        print(f'{self.base_link_x} ({self.base_link_y})')
        x_diff= self.base_link_x-(self.odom_x*math.cos(dyaw)-self.odom_y*math.sin(dyaw)) 
        y_diff= self.base_link_y-(self.odom_x*math.sin(dyaw)+self.odom_y*math.cos(dyaw))
        yaw_diff=dyaw
        if self.slam_x != 0.0 and self.slam_y != 0.0:
            self.tf_publish('map', self.odom_frame, x_diff, y_diff, yaw_diff)      
        try:
            base_link_tf = self.tf_buffer.lookup_transform('map', self.base_frame, rclpy.time.Time())
            base_link_odom= Vector3Stamped()
            base_link_odom.header.stamp = self.get_clock().now().to_msg()
            base_link_odom.header.frame_id = self.odom_frame
            base_link_odom.vector.x = base_link_tf.transform.translation.x
            base_link_odom.vector.y = base_link_tf.transform.translation.y 
            base_link_odom.vector.z = 2 * math.atan2(base_link_tf.transform.rotation.z, base_link_tf.transform.rotation.w)  # 计算yaw
            self.odom_pub.publish(base_link_odom)  # 发布最终车体位置
        except Exception as e:
            return
        
    def tf_publish(self,base_frame:str,child_frame:str,x,y,yaw):
        w = math.cos(yaw / 2)
        z = math.sin(yaw / 2)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = base_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = w
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = z
        try:
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform from {base_frame} to {child_frame}: {e}")
            return
        
    def robot_state_callback(self, msg: String):
        """功能描述：ros2的订阅者的回调函数,接收到消息的时候进行解析，有三个状态，支持'纠正'指令（用Sick数据），'日墙'就去计算slam的坐标系和真实坐标系的一个yaw角的偏差，'reset_slam就认为yaw没有偏差'"""
        """
            参数声明：
        """
        data = json.loads(msg.data)
        if 'correct' in data and data['correct'] is True:
                # 计算Sick数据均值作为真实横向位移
                real_lateral = sum(self.sick_buffer) / len(self.sick_buffer)
                self.get_logger().info(f"真实横向位移（Sick均值）: {real_lateral:.4f}m")
                
                # 获取当前SLAM的TF变换
                tf_now = None
                for map_frame, base_frame in product(self.slam_odom, self.slam_base_link):
                    try:
                        if self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                            tf_now = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
                            break
                    except Exception as e:
                        continue
                if tf_now is None:
                    self.get_logger().error("无法获取SLAM的TF变换！")
                    return
                
                # 当前SLAM的x坐标（纵向）和y坐标（横向）
                slam_x = tf_now.transform.translation.x
                slam_current_y = tf_now.transform.translation.y
                
                # 计算理论横向位移（真实值 - slam_to_map的y偏移补偿）
                theoretical_y = real_lateral - self.slam_to_map[1]
                
                # 计算航向角偏差
                # 理论航向角：基于slam_x和theoretical_y的朝向
                theoretical_yaw = math.atan2(theoretical_y, slam_x)
                # 当前SLAM航向角：基于slam_x和当前y的朝向
                current_slam_yaw = math.atan2(slam_current_y, slam_x)
                # 角度偏差（规范化到[-π, π]）
                self.tf_yaw_diff = theoretical_yaw - current_slam_yaw
                self.get_logger().info(f"角度偏差纠正完成！偏差值: {math.degrees(self.tf_yaw_diff):.2f}°")
                self.sick_buffer.clear()  # 清空缓存，避免重复使用
                
        if 'riqiang' in data:
            if data['riqiang'] == True:
                tf_now=TransformStamped()
                try:
                    tf_now=self.tf_buffer.lookup_transform('camera_init', 'aft_mapped', rclpy.time.Time())
                except Exception as e:
                    self.get_logger().error(f"Failed to lookup transform for riqiang: {e}")
                    return
                #通过y 的误差算出来yaw 的偏移
                x=tf_now.transform.translation.x
                y= self.get_parameter('riqiang_y').value-self.get_parameter('slam_to_map').value[1]
                self.tf_yaw_diff= math.atan2(y,x)-math.atan2(tf_now.transform.translation.y, x)
                print(f"slam 坐标系yaw 当前值{tf_now.transform.translation.y} 理论值{y}")
                print(f"\033[95m日墙角度误差:{self.tf_yaw_diff}\033[0m")
        if 'reset_slam' in data:
            if data['reset_slam'] == True:
                self.tf_yaw_diff = 0.0
                self.tf_yaw_diff =self.get_parameter('loc_to_map').value[2] # 用于存储yaw的均值滤波

def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = fusion_node_t()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    print("Starting fusion node...")
    main()