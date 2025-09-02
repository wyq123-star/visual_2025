import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import matplotlib as mpl

class PointMatcher:
    def __init__(self):
        # 定义源坐标系中的四个点 (x, y)
        x_bias=(0.357+0.13255)
        y_bias=-(0.357+0.3288)
        #假设左右对称,前后不对称
        x_base_bias=0.39 #车体中心距离地图左下角偏移 右手系
        y_base_bias=-0.357 #车体中心距离地图
        laser_x_bias=0.13255 #激光雷达偏移
        laser_y_bias=-0.3288 #激光雷达偏移
        # left_back_point=[0.0+x_base_bias+laser_x_bias, 0.0+y_base_bias+laser_y_bias] #左下
        # right_back_point=[0.0+x_base_bias+laser_x_bias, -8.0-y_base_bias+laser_y_bias] #右下
        # right_front_point=[15.0-x_base_bias-laser_x_bias, -8.0-y_base_bias-laser_y_bias] #右上
        # left_front_point=[15.0-x_base_bias-laser_x_bias, 0.0+y_base_bias-laser_y_bias] #左上
        '''
        雷达贴边的点
        '''
        left_back_point=[-0.032415,0.082415] #左下
        right_back_point=[-0.032415,-8.082415] #右下
        right_front_point=[15.01785,-8.082415] #右上
        left_front_point=[15.01785,0.082415] #左上
        self.target_points = [
            left_back_point,    # 左下
            right_back_point,   # 右下
            right_front_point,  # 右上
            left_front_point     # 左上
        ]
        
        # 定义目标坐标系中的四个点 (x, y)
        self.source_points = [
            [-0.083, 0.02],   # 左下
            [-0.36, -8.183],   # 右下
            [14.635, -8.685],   # 右上
            [14.943, -0.566]    # 左上
        ]
        
        # 存储变换参数 (tx, ty, theta, scale)
        self.transform_params = None
        self.transformed_points = None
        self._calculate_side_lengths()
        
    def _calculate_side_lengths(self):
        """计算并打印两个坐标系的边长"""
        # 计算源坐标系边长
        src_bl = np.array(self.source_points[0])  # 左下
        src_br = np.array(self.source_points[1])  # 右下
        src_tr = np.array(self.source_points[2])  # 右上
        src_tl = np.array(self.source_points[3])  # 左上
        
        self.src_width = np.linalg.norm(src_bl - src_br)  # 短边（垂直边）
        self.src_height = np.linalg.norm(src_bl - src_tl) # 长边（水平边）
        
        # 计算目标坐标系边长
        tgt_bl = np.array(self.target_points[0])
        tgt_br = np.array(self.target_points[1])
        tgt_tr = np.array(self.target_points[2])
        tgt_tl = np.array(self.target_points[3])
        
        self.tgt_width = np.linalg.norm(tgt_bl - tgt_br)   # 短边
        self.tgt_height = np.linalg.norm(tgt_bl - tgt_tl)  # 长边
        
        # 打印结果
        print("\n=== 坐标系边长计算 ===")
        print("【源坐标系】")
        print(f"长边（水平）长度: {self.src_height:.4f} 单位")
        print(f"短边（垂直）长度: {self.src_width:.4f} 单位")
        print(f"长宽比: {self.src_height/self.src_width:.4f}")
        
        print("\n【目标坐标系】")
        print(f"长边长度: {self.tgt_height:.4f} 单位")
        print(f"短边长度: {self.tgt_width:.4f} 单位")
        print(f"长宽比: {self.tgt_height/self.tgt_width:.4f}")
        
    def transform_point(self, point, params):
        """应用仿射变换到单个点"""
        tx, ty, theta, scale = params
        x, y = point
        
        # 转换为弧度
        theta_rad = np.deg2rad(theta)
        
        # 应用旋转、缩放和平移
        x_new = scale * (x * np.cos(theta_rad) - y * np.sin(theta_rad)) + tx
        y_new = scale * (x * np.sin(theta_rad) + y * np.cos(theta_rad)) + ty
        
        return [x_new, y_new]
    
    def loss_function(self, params):
        """计算当前变换参数下的总误差"""
        total_error = 0
        for src, tgt in zip(self.source_points, self.target_points):
            transformed = self.transform_point(src, params)
            # 位置误差 (欧氏距离)
            pos_error = np.sqrt((transformed[0] - tgt[0])**2 + (transformed[1] - tgt[1])**2)
            total_error += pos_error
        return total_error
    
    def match_points(self):
        """执行匹配优化"""
        # 初始猜测 (tx, ty, theta, scale)
        initial_guess = [0, 0, 0, 1]
        
        # 设置边界约束
        bounds = [(-10, 10), (-10, 10), (-180, 180), (0.5, 1.5)]
        
        # 最小化损失函数
        result = minimize(self.loss_function, initial_guess, bounds=bounds, method='L-BFGS-B')
        
        # 保存最优参数
        self.transform_params = result.x
        self.theta_rad= np.deg2rad(self.transform_params[2])
        print(f"匹配参数: tx={result.x[0]:.4f}, ty={result.x[1]:.4f}, "
              f"θ={result.x[2]:.4f}°, scale={result.x[3]:.4f},theta_rad={self.theta_rad:.8f} rad")
        
        # 应用变换到所有点
        self.transformed_points = [self.transform_point(p, self.transform_params) 
                                  for p in self.source_points]
        return self.transform_params
    
    def visualize(self):
        """可视化源点、目标点和变换后的点"""
        # 设置绘图参数
        plt.figure(figsize=(10, 8))
        plt.grid(True)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis('equal')
        
        # 创建颜色列表
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        
        # 绘制源点
        for i, (x, y) in enumerate(self.source_points):
            plt.scatter(x, y, s=120, c=colors[i], marker='o', edgecolors='k', label=f'source{i+1}')
        
        # 绘制目标点
        for i, (x, y) in enumerate(self.target_points):
            plt.scatter(x, y, s=120, c=colors[i], marker='s', edgecolors='k', label=f'target{i+1}')
        
        # 绘制变换后的点
        if self.transformed_points:
            for i, (x, y) in enumerate(self.transformed_points):
                plt.scatter(x, y, s=120, c=colors[i], marker='^', edgecolors='k', label=f'transform{i+1}')
        
        # 添加图例并避免重复
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), loc='best')
        
        plt.tight_layout()
        plt.show()

# 使用示例
if __name__ == "__main__":
    matcher = PointMatcher()
    print("正在计算匹配参数...")
    params = matcher.match_points()
    
    print("\n源点:")
    for i, p in enumerate(matcher.source_points):
        print(f"点{i+1}: x={p[0]:.2f}, y={p[1]:.2f}")
    
    print("\n目标点:")
    for i, p in enumerate(matcher.target_points):
        print(f"点{i+1}: x={p[0]:.2f}, y={p[1]:.2f}")
    
    print("\n变换后的点:")
    for i, p in enumerate(matcher.transformed_points):
        print(f"点{i+1}: x={p[0]:.2f}, y={p[1]:.2f}")
    
    print("\n正在生成可视化...")
    matcher.visualize()