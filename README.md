# 功能介绍

hand_lmk_detection package是使用hobot_dnn package开发的人手关键点检测算法示例，在地平线X3开发板上使用算法模型和图片数据利用BPU处理器进行模型推理。

示例订阅包含人手框信息的ai msg和图片数据image msg，发布包含人手关键点信息的ai msg，用户可以订阅发布的ai msg用于应用开发。

# 编译

## 依赖库


ros package：

- dnn_node
- ai_msgs
- hbm_img_msgs

dnn_node是在地平线X3开发板上利用BPU处理器进行模型推理的pkg，定义在hobot_dnn中。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

hbm_img_msgs为自定义的图片消息格式，用于shared mem场景下的图片传输，hbm_img_msgs pkg定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select hand_lmk_detection`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hand_lmk_detection \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- mono2d_body_detection package：发布人体、人头、人脸、人手框和人体关键点感知msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名                 | 类型        | 解释                                                                                                                | 是否必须 | 支持的配置           | 默认值                       |
| ---------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------- | -------- | -------------------- | ---------------------------- |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式                                                                         | 否       | 0/1                  | 0                            |
| model_file_name        | std::string | 推理使用的模型文件                                                                                                  | 否       | 根据实际模型路径配置 | config/handLMKs.hbm          |
| is_shared_mem_sub      | int         | 是否使用shared mem通信方式订阅图片消息。打开和关闭shared mem通信方式订阅图片的topic名分别为/hbmem_img和/image_raw。 | 0/1      | 0/1                  | 0                            |
| ai_msg_pub_topic_name  | std::string | 发布包含人手关键点检测结果的AI消息的topic名                                                                         | 否       | 根据实际部署环境配置 | /hobot_hand_lmk_detection    |
| ai_msg_sub_topic_name_ | std::string | 订阅包含人手框检测结果的AI消息的topic名                                                                             | 否       | 根据实际部署环境配置 | /hobot_mono2d_body_detection |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .

# 启动图片发布pkg
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
ros2 run mono2d_body_detection mono2d_body_detection --ros-args --log-level error &
# 启动web展示pkg
ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_hand_lmk_detection --log-level error &

# 启动人手关键点检测pkg
ros2 run hand_lmk_detection hand_lmk_detection

```

### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args --log-level error &
# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_hand_lmk_detection --log-level error &

# 启动人手关键点检测pkg
./install/lib/hand_lmk_detection/hand_lmk_detection

```

## 注意事项

第一次运行web展示需要启动webserver服务，运行方法为:

- cd 到websocket的部署路径下：`cd install/lib/websocket/webservice/`（如果是板端编译（无--merge-install编译选项）执行命令为`cd install/websocket/lib/websocket/webservice`）
- 启动nginx：`chmod +x ./sbin/nginx && ./sbin/nginx -p .`

# 结果分析

## X3结果展示

```
[INFO] [1652165932.746271088] [hand lmk ai msg sub]: Recved ai msg, frame_id: 6810, stamp: 1652165932_708693931
[INFO] [1652165932.753882175] [hand lmk det node]: Recved img encoding: nv12, h: 544, w: 960, step: 960, index: 6810, stamp: 1652165932_708693931, data size: 777600
[INFO] [1652165932.755425758] [hand lmk det node]: inputs.size(): 2, rois->size(): 2
[WARN] [1652165932.771337247] [hand lmk det node]: Output from, frame_id: 6810, stamp: 1652165932_708693931, hand rois size: 2, hand rois idx size: 2, hand outputs size: 2, hand lmk size: 2
[INFO] [1652165932.771833234] [hand lmk det node]: target id: 1, rois size: 1 points size: 1  roi type: body  point type: body_kps
[INFO] [1652165932.771959522] [hand lmk det node]: target id: 9, rois size: 1 points size: 0  roi type: head
[INFO] [1652165932.772050812] [hand lmk det node]: target id: 10, rois size: 1 points size: 0  roi type: face
[INFO] [1652165932.772144309] [hand lmk det node]: target id: 18, rois size: 1 points size: 1  roi type: hand  point type: hand_kps
[INFO] [1652165932.772233015] [hand lmk det node]: target id: 13, rois size: 1 points size: 1  roi type: hand  point type: hand_kps
```

以上log显示，订阅到了nv12 encoding格式的图片，和包含人手检测框的ai msg，算法输出人手关键点感知结果。

## web效果展示

# 常见问题

