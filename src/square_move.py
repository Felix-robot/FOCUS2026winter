#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def move_square():
    rospy.init_node('square_patrol', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # === 调试后的参数 ===
    move_cmd = Twist()
    # 【修复方向】：如果之前 x 是横移，这里试着改用 y，或者 x/y 互换
    # 请根据实际情况取消注释下面其中一行：
    move_cmd.linear.x = 0.5   # 方案A：如果这行是前进
    # move_cmd.linear.y = 0.4 # 方案B：如果上面那行是横移，试试这行
    
    turn_cmd = Twist()
    turn_cmd.angular.z = 0.6 # 转向速度

    stop_cmd = Twist()

    print("--- 启动校准版正方形巡逻 ---")
    
    for i in range(4):
        # 1. 前进 (时间加长到 5秒)
        print(f"边 {i+1}: 前进...")
        start_time = time.time()
        while time.time() - start_time < 5.0: 
            pub.publish(move_cmd)
            rate.sleep()
        
        # 停顿
        pub.publish(stop_cmd)
        time.sleep(0.5)

        # 2. 转向 (时间加倍到 5.5秒)
        print(f"边 {i+1}: 转向...")
        start_time = time.time()
        while time.time() - start_time < 5.5: 
            pub.publish(turn_cmd)
            rate.sleep()
            
        pub.publish(stop_cmd)
        time.sleep(0.5)

    print("任务完成")

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass