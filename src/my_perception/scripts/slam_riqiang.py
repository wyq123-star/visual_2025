#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer
from geometry_msgs.msg import TransformStamped,Vector3Stamped
import json,os,math,numpy as np
import rclpy.time
from std_msgs.msg import String
from itertools import product
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer,StaticTransformBroadcaster
class fusion_node_t(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('odom_frame','odom_wheel')  #地图坐标系下轮式里程计坐标,也就是根据slam融合纠正过累积误差的坐标
        self.declare_parameter('base_frame', 'base_link') # 地图坐标系下融合码盘的base_link坐标
        self.declare_parameter('slam_odom',['camera_init']) # 被监听的tf地图坐标 
        self.declare_parameter('slam_base_link',['body','aft_mapped'])  # 被监听的tf基座坐标
        self.declare_parameter('odom_topic','/odom')   #轮式里程计
        self.declare_parameter('base_to_laser', [0.085,0.095, 0.0])  # 激光雷达到base_link的偏移 右手系
        self.declare_parameter('riqiang_y', -0.10975) #日墙时候的雷达y偏移
        self.declare_parameter('slam_to_map',[0.46876+0.26775,-0.08475-0.0815,0.0])
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value #轮式里程计坐标
        self.base_frame = self.get_parameter('base_frame').value #发布的base_link坐标
        self.slam_to_map = self.get_parameter('slam_to_map').value
        self.base_to_laser = self.get_parameter('base_to_laser').value  # [x_offset, y_offset, yaw_offset]
        self.slam_odom = self.get_parameter('slam_odom').value  #被监听的tf地图坐标
        self.slam_base_link = self.get_parameter('slam_base_link').value  #被监听
        
        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.x_diff,self.y_diff,self.yaw_diff = 0.0,0.0,0.0

        #两个定时器回调和两个订阅者回调
        self.fuse_timer = self.create_timer(0.1,self.fuse_callback)
        self.slam_timer = self.create_timer(0.01,self.slam_tf_callback)
        self.odom_callback_sub= self.create_subscription(Vector3Stamped, self.odom_topic, self.odom_callback, 1)
        self.odom_pub= self.create_publisher(Vector3Stamped, 'base_link_odom', 10)
        self.robot_sub= self.create_subscription(String, 'robot_state', self.robot_state_callback, 1)

    def slam_tf_callback(self):
        transform=TransformStamped()
        if len(self.slam_odom) >0 and len(self.slam_base_link) > 0:
            #尝试找出其中能用的tf
            for map_frame, base_frame in product(self.slam_odom, self.slam_base_link): #遍历所有frame组合
                try:
                    if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                        continue
                    transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
                    break  # 找到一个可用的就退出循环
                except Exception as e:
                    self.get_logger().error(f"Failed to get transform from {map_frame} to {base_frame}: {e}")
        else:
            try:
                if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                    return
                transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"Failed to get transform: {e}")
                return

        try:
            # 原始坐标
            original_x = transform.transform.translation.x
            original_y = transform.transform.translation.y
            original_z = transform.transform.translation.z
            self.slam_x = original_y*math.sin(self.base_to_laser[2]) + original_x*math.cos(self.base_to_laser[2])
            self.slam_y = original_y*math.cos(self.base_to_laser[2]) - original_x*math.sin(self.base_to_laser[2])
            # 处理姿态（四元数转偏航角）
            orientation = transform.transform.rotation
            siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            original_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # 应用旋转角度到偏航角（将角度转换为弧度）
            rotation_rad = math.radians(self.base_to_laser[2])
            self.slam_yaw = rotation_rad + original_yaw
            
            # 规范化偏航角到[-π, π]范围
            if self.slam_yaw > math.pi:
                self.slam_yaw -= 2 * math.pi
            elif self.slam_yaw < -math.pi:
                self.slam_yaw += 2 * math.pi

            self.tf_publish('map', self.odom_frame, self.x_diff,self.y_diff, self.yaw_diff) 
            
        except Exception as e:
            self.get_logger().error(f"SLAM坐标转换错误: {e}")

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

    def odom_callback(self,msg:Vector3Stamped):
        self.odom_x = msg.vector.x
        self.odom_y = msg.vector.y
        self.odom_yaw = msg.vector.z
        self.tf_publish(self.odom_frame, self.base_frame, self.odom_x, self.odom_y, self.odom_yaw)

    def fuse_callback(self):
        dyaw= self.slam_yaw - self.odom_yaw
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi
        self.base_link_x=self.slam_x - self.r*math.sin(self.laser_angle + self.slam_yaw) +self.base_to_laser[1]
        self.base_link_y=self.slam_y + self.r*math.cos(self.laser_angle + self.slam_yaw) -self.base_to_laser[0]
        self.x_diff= self.base_link_x-(self.odom_x*math.cos(dyaw)-self.odom_y*math.sin(dyaw)) 
        self.y_diff= self.base_link_y-(self.odom_x*math.sin(dyaw)+self.odom_y*math.cos(dyaw))
        self.yaw_diff=dyaw    

       
    def robot_state_callback(self, msg: String):
        """功能描述：ros2的订阅者的回调函数,接收到消息的时候进行解析，有两个状态，'日墙'就去计算slam的坐标系和真实坐标系的一个yaw角的偏差，'reset_slam就认为yaw没有偏差'"""
        """
            参数声明：
        """
        data = json.loads(msg.data)
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