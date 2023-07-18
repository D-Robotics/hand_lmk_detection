# 功能介绍

人手关键点检测算法示例订阅图片和包含人手框信息的智能msg，利用BPU进行算法推理，发布包含人手关键点信息的算法msg。

人手关键点索引如下图：

![](./imgs/hand_lmk_index.jpeg)

代码仓库：<https://github.com/HorizonRDK/hobot_vio.git>

# 物料清单

| 机器人名称          | 生产厂家 | 参考链接                                                     |
| :------------------ | -------- | ------------------------------------------------------------ |
| RDK X3             | 多厂家 | [点击跳转](https://developer.horizon.ai/sunrise) |
| camera             | 多厂家 | MIPI cam:[F37 200W像素](https://detail.tmall.com/item.htm?abbucket=12&id=683310105141&ns=1&spm=a230r.1.14.28.1dd135f0wI2LwA&skuId=4897731532963)/[GC4663 400W像素](https://detail.tmall.com/item.htm?abbucket=12&id=683310105141&ns=1&spm=a230r.1.14.28.1dd135f0wI2LwA&skuId=4897731532963)/[IMX219 800W像素](https://detail.tmall.com/item.htm?abbucket=9&id=710344235988&rn=259e73f46059c2e6fc9de133ba9ddddf&spm=a1z10.5-b-s.w4011-22651484606.159.55df6a83NWrGPi)或usb cam|


# 使用方法

## 准备工作

- 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像
- 摄像头正确连接到RDK X3

# 使用方法

**1.安装功能包**

启动机器人后，通过终端或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

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

打开同一网络电脑的浏览器，访问IP地址（浏览器输入http://IP:8000，IP为地平线RDK的IP地址），即可看到视觉识别的实时效果。

# 接口说明

## 参数

| 参数名                 | 类型        | 解释                                                                                                                | 是否必须 | 支持的配置           | 默认值                       |
| ---------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------- | -------- | -------------------- | ---------------------------- |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式                                                                         | 否       | 0/1                  | 0                            |
| model_file_name        | std::string | 推理使用的模型文件                                                                                                  | 否       | 根据实际模型路径配置 | config/handLMKs.hbm          |
| is_shared_mem_sub      | int         | 是否使用shared mem通信方式订阅图片消息。打开和关闭shared mem通信方式订阅图片的topic名分别为/hbmem_img和/image_raw。 | 0/1      | 0/1                  | 0                            |
| ai_msg_pub_topic_name  | std::string | 发布包含人手关键点检测结果的AI消息的topic名                                                                         | 否       | 根据实际部署环境配置 | /hobot_hand_lmk_detection    |
| ai_msg_sub_topic_name_ | std::string | 订阅包含人手框检测结果的AI消息的topic名                                                                             | 否       | 根据实际部署环境配置 | /hobot_mono2d_body_detection |


# 参考资料


# 常见问题

