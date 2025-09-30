# 25赛季开放轮式_海底梅有可然宾_上位机代码
HDURobotStar 25赛季开放轮式格斗上位机代码  
## 项目概述

这是一个基于ROS (Robot Operating System) 的机器人视觉导航项目，主要用于中国高校智能机器人创意大赛主题三轮式格斗竞赛环境。项目集成了AprilTag视觉识别、激光雷达目标检测与跟踪、以及串口通信控制功能，实现智能机器人的自主导航和任务执行。

## 主要功能

### 1. AprilTag 视觉识别系统
- **炸弹块识别**: 通过AprilTag ID=0 识别炸弹块，实现避障功能
- **能量块识别**: 通过AprilTag ID=1 识别能量块，实现目标跟踪
- **绿色方块检测**: 基于HSV颜色空间的绿色目标检测
- **梯形掩码区域限制**: 限制检测区域，提高识别精度
- **PID控制算法**: 实现精确的目标跟踪控制

### 2. 激光雷达目标检测与跟踪 (DATMO)
- **移动目标检测**: 基于2D激光雷达的移动目标检测
- **目标跟踪**: 使用卡尔曼滤波进行目标状态估计
- **矩形拟合**: 对检测到的目标进行矩形拟合
- **L型角点提取**: 提取目标的关键角点信息
- **数据关联**: 多帧间的目标关联算法

### 3. 串口通信系统
- **PWM数据发送**: 向底层控制器发送电机PWM控制信号
- **激光测距数据**: 集成双激光测距传感器数据
- **数据包协议**: 自定义12字节数据包格式
- **实时控制**: 支持实时控制指令传输

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   摄像头输入    │    │   激光雷达      │    │   激光测距      │
│   (USB Camera)  │    │   (LDLiDAR)     │    │   (双激光)      │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          ▼                      ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  AprilTag检测   │    │   DATMO检测     │    │   距离传感器    │
│   (apriltag_    │    │   (datmo)       │    │   数据处理      │
│   detection)    │    │                 │    │                 │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          ▼                      ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    控制决策融合                                │
│              (apriltag_node.py)                               │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    串口通信节点                                │
│                  (serial_node.cpp)                            │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    底层控制器                                  │
│                  (PWM控制)                                    │
└─────────────────────────────────────────────────────────────────┘
```

## 项目结构

```
apriltag_ws/
├── src/
│   ├── apriltag_detection/          # AprilTag检测包
│   │   ├── scripts/
│   │   │   └── apriltag_node.py     # 主控制节点
│   │   ├── src/
│   │   │   └── serial_node.cpp      # 串口通信节点
│   │   └── msg/
│   │       └── AprilTagDistance.msg # 自定义消息类型
│   ├── datmo-master/                # 激光雷达目标检测包
│   │   ├── src/                     # 核心算法实现
│   │   ├── msg/                     # 自定义消息类型
│   │   └── launch/                  # 启动文件
│   └── ros_app/
│       └── src/ldlidar/             # 激光雷达驱动
│           ├── src/ros_node/        # ROS节点实现
│           └── launch/              # 启动文件
├── build/                           # 编译目录
├── devel/                           # 开发环境
└── README.md                        # 项目说明文档
```

## 依赖环境

### 系统要求
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- OpenCV 4.x
- CMake 3.16+

### ROS包依赖
- `cv_bridge` - OpenCV与ROS图像消息转换
- `sensor_msgs` - 传感器消息类型
- `std_msgs` - 标准消息类型
- `serial` - 串口通信
- `roscpp` - C++ ROS客户端库
- `rospy` - Python ROS客户端库
- `tf` - 坐标变换
- `nav_msgs` - 导航消息类型
- `visualization_msgs` - 可视化消息类型

### Python依赖
- `apriltag` - AprilTag检测库
- `numpy` - 数值计算
- `opencv-python` - 计算机视觉库

## 安装与编译

### 1. 克隆项目
```bash
cd ~/catkin_ws/src
git clone <repository_url> apriltag_ws
cd apriltag_ws
```

### 2. 安装依赖
```bash
# 安装ROS依赖
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-serial
sudo apt-get install ros-noetic-tf
sudo apt-get install ros-noetic-nav-msgs
sudo apt-get install ros-noetic-visualization-msgs

# 安装Python依赖
pip3 install apriltag
pip3 install opencv-python
pip3 install numpy
```

### 3. 编译项目
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 启动激光雷达
```bash
roslaunch ldlidar stp23.launch
```

### 2. 启动目标检测
```bash
roslaunch datmo datmo.launch
```

### 3. 启动主控制节点
```bash
rosrun apriltag_detection apriltag_node.py
```

### 4. 启动串口通信
```bash
rosrun apriltag_detection serial_node
```

## 配置参数

### AprilTag检测参数
- `BASE_PWM`: 基础PWM值 (16)
- `MAX_PWM`: 最大PWM值 (28)
- `GREEN_LOWER/UPER`: 绿色检测HSV阈值
- `MIN_GREEN_AREA`: 最小绿色区域面积
- `MAX_ASPECT_RATIO`: 最大长宽比

### 激光雷达参数
- `threshold_distance`: 聚类距离阈值 (0.3m)
- `euclidean_distance`: 欧几里得距离阈值 (0.35m)
- `max_cluster_size`: 最大聚类大小 (90)
- `max_length/width`: 目标最大长度/宽度

### 串口通信参数
- 串口设备: `/dev/base`
- 波特率: 115200
- 数据包格式: 12字节协议

## 数据包协议

串口通信使用12字节数据包格式：

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 0x7B | 帧头 |
| 1-2 | 左轮PWM | 16位整数 |
| 3-4 | 右轮PWM | 16位整数 |
| 5-6 | 激光1距离 | 16位整数(mm) |
| 7-8 | 激光2距离 | 16位整数(mm) |
| 9 | 校验和 | 前9字节异或 |
| 10 | 标签ID | AprilTag检测结果 |
| 11 | 0x7D | 帧尾 |

## 控制策略

### 优先级控制
1. **避障模式**: 检测到炸弹块(ID=0)时优先避障
2. **激光控制**: 距离过近时使用激光测距控制
3. **目标跟踪**: 检测到能量块(ID=1)或绿色方块时进行跟踪
4. **默认模式**: 无目标时保持基础PWM运行

### PID控制参数
- **AprilTag跟踪**: Kp=0.0156, Ki=0.0, Kd=0.0
- **绿色方块跟踪**: Kp=0.0152, Ki=0.0, Kd=0.0  
- **激光控制**: Kp=0.0045, Ki=0.0, Kd=0.0

## 调试与可视化

### 话题列表
- `/laser` - 激光雷达数据
- `/laser2` - 第二个激光测距数据
- `/wheel_pwm` - PWM控制数据
- `/yes` - AprilTag检测结果
- `/datmo/theta` - 目标角度信息
- `/theta` - 融合后的角度信息

### 可视化工具
- 使用RViz查看激光雷达数据
- 使用`rqt_graph`查看节点连接
- 使用`rostopic echo`查看话题数据

## 故障排除

### 常见问题
1. **摄像头无法打开**: 检查USB摄像头权限和驱动
2. **串口无法打开**: 检查设备路径和权限
3. **激光雷达无数据**: 检查串口连接和波特率设置
4. **AprilTag检测失败**: 检查光照条件和标签质量

### 调试命令
```bash
# 检查设备权限
ls -l /dev/ttyUSB*
ls -l /dev/video*

# 检查ROS话题
rostopic list
rostopic echo /laser

# 检查节点状态
rosnode list
rosnode info /apriltag_detector
```

## 开发团队

- 维护者: MJJjacky
- 开发环境: Ubuntu 20.04 + ROS Noetic
- 项目类型: 机器人竞赛项目

## 许可证



## 更新日志

- v0.0.0: 初始版本，实现基础AprilTag检测和串口通信功能
- 集成DATMO激光雷达目标检测
- 添加多传感器融合控制策略
- 优化PID控制参数和避障算法

---

**注意**: 本项目专为特定机器人竞赛环境设计，使用前请根据实际硬件配置调整相关参数。
