# 功能介绍

人手关键点检测算法示例订阅图片和包含人手框信息的智能msg，利用BPU进行算法推理，发布包含人手关键点信息的算法msg。

人手关键点索引如下图：

![](./imgs/hand_lmk_index.jpeg)


# 物料清单

| 物料名称            | 生产厂家 | 参考链接                                                     |
| :------------------ | -------- | ------------------------------------------------------------ |
| RDK X3 / RDK Ultra  | 多厂家 | [RDK X3](https://developer.horizon.cc/rdkx3)<br>[RDK Ultra](https://developer.horizon.cc/rdkultra) |
| camera              | 多厂家 | [MIPI相机](https://developer.horizon.cc/nodehubdetail/168958376283445781)<br>[USB相机](https://developer.horizon.cc/nodehubdetail/168958376283445777)|


# 准备工作

- 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像
- 摄像头正确连接到RDK X3

# 使用方法


**1.安装功能包**

启动机器人后，通过SSH终端或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-hand-lmk-detection
```

**2.运行人手关键点检测功能**

**使用MIPI摄像头发布图片**

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/lib/hand_lmk_detection/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

**使用USB摄像头发布图片**

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/lib/hand_lmk_detection/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

**3.查看效果**

打开同一网络电脑的浏览器，访问[http://IP:8000](http://IP:8000)即可看到视觉识别的实时效果，其中IP为RDK的IP地址:
![](./imgs/hand_lmk_render.jpg)

# 接口说明

## 话题

手关键点检测结果都通过[hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/HorizonRDK/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)话题发布，该话题的详细定义如下：
```shell
# 感知结果

# 消息头
std_msgs/Header header

# 感知结果的处理帧率
# fps val is invalid if fps is less than 0
int16 fps

# 性能统计信息，比如记录每个模型推理的耗时
Perf[] perfs

# 感知目标集合
Target[] targets

# 消失目标集合
Target[] disappeared_targets
```

| 名称                 | 消息类型        | 说明|
| ---------------------- | ----------- |---------------------------- |
| /hobot_hand_lmk_detection | [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/HorizonRDK/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)  | 发布识别到的手关键点信息（开启手势唤醒之后才会出现） |
| /hobot_mono2d_body_detection          | [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/HorizonRDK/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)   | 订阅前一个node识别到的人体目标信息，包括人体框、人脸框、手框、人体关键点 |
| /hbmem_img | [hobot_msgs/hbm_img_msgs/msg/HbmMsg1080P](https://github.com/HorizonRDK/hobot_msgs/blob/develop/hbm_img_msgs/msg/HbmMsg1080P.msg)  | 当is_shared_mem_sub == 1时，用shared mem通信方式订阅上一个node发布图像数据|
| /image_raw | hsensor_msgs/msg/Image  |  当is_shared_mem_sub == 0时，订阅用ros的普通方式订阅上一个node发布相关的图像数据|


## 参数

| 参数名                 | 类型        | 解释                                                                                                                | 是否必须 | 支持的配置           | 默认值                       |
| ---------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------- | -------- | -------------------- | ---------------------------- |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式                                                                         | 否       | 0/1                  | 0                            |
| model_file_name        | std::string | 推理使用的模型文件                                                                                                  | 否       | 根据实际模型路径配置 | config/handLMKs.hbm          |
| is_shared_mem_sub      | int         | 是否使用shared mem通信方式订阅图片消息。打开和关闭shared mem通信方式订阅图片的topic名分别为/hbmem_img和/image_raw。 | 0/1      | 0/1                  | 0                            |
| ai_msg_pub_topic_name  | std::string | 发布包含人手关键点检测结果消息的topic名                                                                         | 否       | 根据实际部署环境配置 | /hobot_hand_lmk_detection    |
| ai_msg_sub_topic_name_ | std::string | 订阅包含人手框检测结果的AI消息的topic名                                                                             | 否       | 根据实际部署环境配置 | /hobot_mono2d_body_detection |

