#!/usr/bin/env python3

import cv2
import apriltag
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
import time
import threading
from sensor_msgs.msg import Range

class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        dt = max(dt, 1e-5)
        
        proportional = self.kp * error
        self.integral += error * dt
        self.integral = max(min(self.integral, 25), -25)
        derivative = (error - self.last_error) / dt
        
        output = proportional + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), -self.max_output)
        
        self.last_error = error
        self.last_time = current_time
        return output


# 全局变量存储激光数据
laser1_distance = None
laser2_distance = None
lock = threading.Lock()

def laser1_callback(msg):
    global laser1_distance
    with lock:
        # 将米转换为毫米
        laser1_distance = msg.range * 1000  # 1m = 1000mm
        rospy.logdebug(f"右激光更新: {laser1_distance:.1f}mm")

def laser2_callback(msg):
    global laser2_distance
    with lock:
        # 将米转换为毫米 
        laser2_distance = msg.range * 1000  # 1m = 1000mm
        rospy.logdebug(f"左激光更新: {laser2_distance:.1f}mm")
def theta_callback(msg):
    global theta
    with lock:
       theta=msg.data
rospy.Subscriber("/laser", Range, laser1_callback)
rospy.Subscriber("/laser2", Range, laser2_callback)
rospy.Subscriber("theta", Float32, theta_callback)

rospy.init_node("apriltag_detector", anonymous=True)
tag_pub = rospy.Publisher("/yes", Float32MultiArray, queue_size=10)
pwm_pub = rospy.Publisher("/wheel_pwm", Float32MultiArray, queue_size=10)

options = apriltag.DetectorOptions(families='tag36h11')
detector = apriltag.Detector(options)

pid = PIDController(kp=0.0156, ki=0.00, kd=0.0, max_output=25)
pid_green = PIDController(kp=0.0152, ki=0.00, kd=0.0, max_output=25)
pid_laser = PIDController(kp=0.0045, ki=0.0, kd=0.00, max_output=22)  # 调整激光控制参数
BASE_PWM = 16
MAX_PWM = 28
BASE_PWM_MY = 6
BASE_PWM_NY = 11

cap = cv2.VideoCapture(0)

cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))# 视频流格式
cap.set(5, 120);# 帧率
cap.set(3, 800)# 帧宽
cap.set(4, 600)# 帧高
# 获取相机宽高以及帧率
width = cap.get(3)
height  = cap.get(4)
frame = cap.get(5) #帧率只对视频有效，因此返回值为0
#打印信息
print(width ,height)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cam_center_x = frame_width // 2
cam_center_y = frame_height // 2

# 定义梯形掩码顶点（比例坐标）
trapezoid_pts = np.array([
    [int(0.1 * frame_width), frame_height],          # 左下
    [int(0.3 * frame_width), int(0.2 * frame_height)], # 左上
    [int(0.7 * frame_width), int(0.2 * frame_height)], # 右上
    [int(0.9 * frame_width), frame_height]            # 右下
], dtype=np.int32)

# 绿色HSV阈值范围（可根据实际情况调整）
GREEN_LOWER = np.array([42, 80, 120])
GREEN_UPPER = np.array([75, 210,255])


# 遮挡检测参数
OCCLUDE_BRIGHTNESS_THRESH = 20     # 亮度阈值（0-255）
OCCLUDE_SKIN_RATIO = 0.9         # 皮肤色区域占比阈值
# 皮肤色HSV范围（根据实际手部颜色调整）
SKIN_LOWER = np.array([0, 0, 0])    # 低阈值（H,S,V）
SKIN_UPPER = np.array([180, 255, 46]) # 高阈值

# 绿色方块识别参数
MIN_GREEN_AREA = 2 * 20 * 0.8  # 最小面积阈值 (像素)
MAX_ASPECT_RATIO =2.5         # 最大长宽比


JS = 0

while not rospy.is_shutdown():
    if cap.isOpened():
        ret, frame = cap.read()
        frame=frame[:,:]
        if not ret:
            continue    
        # 初始化控制参数
        left_pwm = BASE_PWM
        right_pwm = BASE_PWM
        control_mode = "Vision"
        emergency_stop = False
    
        # 获取激光数据
        with lock:
            current_laser1 = laser1_distance
            current_laser2 = laser2_distance
            
            # 转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 计算图像平均亮度
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.mean(gray)
        # 检测皮肤色区域
        #  skin_mask = cv2.inRange(hsv, SKIN_LOWER, SKIN_UPPER)
         # skin_ratio = np.count_nonzero(skin_mask) / (frame_width * frame_height)
        # 遮挡判断条件
         # is_occluded = (avg_brightness < OCCLUDE_BRIGHTNESS_THRESH) and \
                         # (skin_ratio > OCCLUDE_SKIN_RATIO)
          #if is_occluded:
            # 发布停止指令并跳过后续处理
            #  pwm_msg = Float32MultiArray()
             # pwm_msg.data = [0, 0]
              #pwm_pub.publish(pwm_msg)
             # cv2.putText(frame, "OCCLUDED!", (30, 60),
              #      cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
             # cv2.imshow('Detection', frame)
             # break
        # === 梯形掩码生成 ===
        mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        cv2.fillPoly(mask, [trapezoid_pts], 255)
        # AprilTag检测
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)
    
        max_tag_area = 0
        max_detection = None
        
        tag = 0
        if detections:
             
             for detection in detections:
            # 计算标签中心坐标
                center_x = int((detection.corners[0][0] + detection.corners[2][0]) / 2)
                center_y = int((detection.corners[0][1] + detection.corners[2][1]) / 2)
                
            # 特殊处理ID为0的标签
                if detection.tag_id == 0:
                # 检查是否在掩码区域内
                    if mask[center_y, center_x] == 0:
                        continue  # 跳过区域外检测
                        
            # 计算标签面积
                pts = detection.corners.astype(np.int32).reshape((-1, 1, 2))
                area = cv2.contourArea(pts) * 100.0
                
                # 更新最大面积检测
                if area > max_tag_area:
                    max_tag_area = area
                    max_detection = detection
        using_laser = False
        avoid = False
        if max_detection is not None:
            # 跟踪AprilTag
                detection = max_detection
                tag_id = detection.tag_id
                pts = detection.corners.astype(int)
                center_x = (pts[0][0] + pts[2][0]) // 2
                center_y = (pts[0][1] + pts[2][1]) // 2
                horizontal_offset = center_x - cam_center_x
                if tag_id == 0:
                    avoid = True
                    control_mode = "avoid"
                    if(horizontal_offset>0):
                            left_pwm =  0
                            right_pwm =  BASE_PWM+4
                    else :
                            left_pwm =  BASE_PWM+4
                            right_pwm =  0            
                cv2.polylines(frame, [trapezoid_pts], True, (255, 0, 255), 2)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {tag_id}", (center_x-20, center_y-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)    
    
        if None not in (laser1_distance, laser2_distance) and not avoid:
            min_distance = min(laser1_distance, laser2_distance)
            if min_distance < 380:  # 现在200代表200mm
                using_laser = True
                control_mode = "Laser PID"
                # 激光控制模式
                error = laser1_distance - laser2_distance  # 左 - 右，保持两侧距离平衡
                control = pid_laser.compute(error)
                left_pwm = BASE_PWM +5- control
                right_pwm = BASE_PWM +5+ control
                control_mode = "Laser PID"
                
                # 限制PWM输出范围
                left_pwm = int(max(min(left_pwm, 26), 9))
                right_pwm = int(max(min(right_pwm, 26), 9))
                
        if not using_laser and not avoid:    
                # 绿色方块检测
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
            green_mask = cv2.erode(green_mask, None, iterations=2)
            green_mask = cv2.dilate(green_mask, None, iterations=2)
            contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_green_area = 0
            green_center_x = 0
            green_center_y = 0
            best_green_contour = None
            
           # 绿色方块检测部分修改如下：

            # 使用 boundingRect 代替 minAreaRect
            if contours:
                # 找到最大轮廓
                max_contour = max(contours, key=cv2.contourArea)
                contour_area = cv2.contourArea(max_contour)
                tag = 1
                
                if contour_area > MIN_GREEN_AREA:
                    # 使用 boundingRect 获取边界矩形
                    x, y, w, h = cv2.boundingRect(max_contour)
                    
                    # 计算长宽比（确保不为零）
                    aspect_ratio = max(w, h) / max(min(w, h), 1e-5)
                    
                    # 添加矩形约束：长宽比
                    if aspect_ratio < MAX_ASPECT_RATIO:
                        best_green_contour = max_contour
                        max_green_area = contour_area
                        
                        # 计算中心点
                        green_center_x = x + w // 2
                        green_center_y = y + h // 2
                        
                        # 保存矩形参数用于绘制
                        best_rect = (x, y, w, h)
                        best_aspect_ratio = aspect_ratio
            
                    # 绘制边界矩形
                    if best_green_contour is not None:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        
                        # 显示长宽比信息
                        cv2.putText(frame, f"AR: {best_aspect_ratio:.2f}", (green_center_x + 10, green_center_y + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
        # 目标优先级判断
            track_green = False
            # 使用轮廓面积作为判断依据（max_green_area）
            if max_green_area *100> 2500 and max_tag_area == 0:
                track_green = True
    
            if track_green:
            # 跟踪绿色方块
                control_mode = "track green"
                horizontal_offset = green_center_x - cam_center_x
                control = pid_green.compute(horizontal_offset)
                left_pwm = BASE_PWM  + 2 + control
                right_pwm = BASE_PWM + 2 - control
                left_pwm = int(max(min(left_pwm, 28), 5))
                right_pwm = int(max(min(right_pwm, 28), 5))
                
                # 绘制轮廓和中心点
                if best_green_contour is not None:
                    cv2.circle(frame, (green_center_x, green_center_y), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"Green: {max_green_area:.1f}", (green_center_x+10, green_center_y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    
            elif max_detection is not None:
            # 跟踪AprilTag
                detection = max_detection
                tag_id = detection.tag_id
                pts = detection.corners.astype(int)
                center_x = (pts[0][0] + pts[2][0]) // 2
                center_y = (pts[0][1] + pts[2][1]) // 2
                horizontal_offset = center_x - cam_center_x
    
                if tag_id == 1:
                    tag = 1
                    control = pid.compute(horizontal_offset)
                    left_pwm = BASE_PWM  + 2 + control
                    right_pwm = BASE_PWM + 2 - control
                    left_pwm = int(max(min(left_pwm, 28), 5))
                    right_pwm = int(max(min(right_pwm,28), 5))
                cv2.polylines(frame, [trapezoid_pts], True, (255, 0, 255), 2)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {tag_id}", (center_x-20, center_y-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            else:
            # 无目标
                left_pwm, right_pwm = BASE_PWM , BASE_PWM
        
        pwm_msg = Float32MultiArray()
        tag_msg = Float32MultiArray()
        tag_msg.data = [tag]
        pwm_msg.data = [left_pwm, right_pwm]
        pwm_pub.publish(pwm_msg)
        tag_pub.publish(tag_msg)
        # 显示调试信息
        # debug_text = [
             # f"Control Mode: {control_mode}",
             # f"PWM: L={left_pwm} R={right_pwm}",
             # f"TAG: TAG ={tag}",
             # f"Lasers: L={current_laser2 if current_laser2 is not None else '---'}mm "
             # f"R={current_laser1 if current_laser1 is not None else '---'}mm"
         # ]
        # for i, text in enumerate(debug_text):
             # cv2.putText(frame, text, (10, 30 + 25*i),
                         # cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    
        # cv2.imshow('Detection', frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
          #break 

cap.release()
cv2.destroyAllWindows()