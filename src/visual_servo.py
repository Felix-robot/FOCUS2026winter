#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class VisualServo:
    def __init__(self):
        rospy.init_node('visual_servo_node', anonymous=True)
        
        # === 核心配置区 (根据之前的正方形测试调整) ===
        # 如果你的狗之前 linear.x 是横移，把下面改成 'y'
        self.FORWARD_AXIS = 'x'  
        
        # 目标配置
        self.target_area = 25000  # 目标面积（狗会停在这个距离）
        self.camera_topic = "/unitree/camera/image_raw" 
        
        # PID 参数 (比例系数)
        self.kp_yaw = 0.003   # 转向灵敏度
        self.kp_dist = 0.0001 # 前进灵敏度

        # === 初始化 ===
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.cmd = Twist()
        
        print("视觉伺服启动！请在 Gazebo 中拖动红方块...")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # 1. 图像处理：找红色
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 红色的 HSV 范围 (OpenCV 中 Hue 有两个区间)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        
        # 2. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        height, width, _ = cv_image.shape
        center_x = width / 2
        
        # 默认停止
        forward_speed = 0.0
        turn_speed = 0.0
        found_target = False

        if len(contours) > 0:
            # 找到最大的红色物体
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 500: # 过滤掉噪点
                found_target = True
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2
                
                # === 3. 闭环控制算法 ===
                
                # A. 转向控制 (让红块回到视野中心)
                # 误差 = 屏幕中心 - 物体中心
                error_yaw = center_x - cx
                turn_speed = self.kp_yaw * error_yaw
                
                # B. 前后控制 (根据面积大小判断远近)
                # 误差 = 目标面积 - 当前面积
                error_dist = self.target_area - area
                
                # 限制最大速度，防止暴冲
                raw_speed = self.kp_dist * error_dist
                forward_speed = np.clip(raw_speed, -0.4, 0.4)

                # 可视化：画框
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                text = f"Area: {int(area)} Spd: {forward_speed:.2f}"
                cv2.putText(cv_image, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # === 4. 发送指令 ===
        if found_target:
            if self.FORWARD_AXIS == 'x':
                self.cmd.linear.x = forward_speed
                self.cmd.linear.y = 0.0
            else:
                self.cmd.linear.x = 0.0
                self.cmd.linear.y = forward_speed # 之前横移修正
            
            self.cmd.angular.z = turn_speed
        else:
            # 没看到红色就停下
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
            
        self.cmd_pub.publish(self.cmd)

        # 显示画面 (录屏需要)
        cv2.imshow("Robot Eye - Acceptance Point 4", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    try:
        vs = VisualServo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()