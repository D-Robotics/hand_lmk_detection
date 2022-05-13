// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif

#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "include/ai_msg_sub_node.h"

#ifndef MONO2D_BODY_DET_NODE_H_
#define MONO2D_BODY_DET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::Model;
using hobot::dnn_node::ModelInferTask;
using hobot::dnn_node::ModelManager;
using hobot::dnn_node::ModelRoiInferTask;

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::OutputParser;
using hobot::dnn_node::OutputDescription;

using ai_msgs::msg::PerceptionTargets;

struct HandLmkOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
  // std::shared_ptr<std::vector<hbDNNRoi>> rois;
  // 符合resizer模型限制条件的roi
  std::shared_ptr<std::vector<hbDNNRoi>> valid_rois;
  // 符合resizer模型限制条件的roi对应于原始roi的索引
  // std::vector<size_t> valid_roi_idx;
  // 原始roi的索引对应于valid_rois的索引
  std::map<size_t, size_t> valid_roi_idx;
  ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg;
};

struct FeedbackImgInfo {
  std::string image_ = "config/960x540.nv12";
  int img_w = 960;
  int img_h = 540;
  int32_t roi_left = 350;
  int32_t roi_top = 212;
  int32_t roi_right = 445;
  int32_t roi_bottom = 313;
};

class HandLmkDetNode : public DnnNode {
 public:
  HandLmkDetNode(const std::string &node_name,
                 std::shared_ptr<AiMsgSubNode> ai_msg_sub_node = nullptr,
                 const NodeOptions &options = NodeOptions());
  ~HandLmkDetNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  // 用于预测的图片来源，0：订阅到的image msg；1：本地nv12格式图片
  int feed_type_ = 0;
  FeedbackImgInfo fb_img_info_;

  std::string model_file_name_ = "config/handLMKs.hbm";
  std::string model_name_ = "handLMKs";
  ModelTaskType model_task_type_ = ModelTaskType::ModelRoiInferType;

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  int32_t model_output_count_ = 1;
  const int32_t kps_output_index_ = 0;

  int is_sync_mode_ = 1;

  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 1;

  int dump_render_img_ = 0;

  std::chrono::high_resolution_clock::time_point output_tp_;
  int output_frameCount_ = 0;
  int smart_fps_ = 0;
  std::mutex frame_stat_mtx_;

  std::string ai_msg_pub_topic_name = "/hobot_hand_lmk_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  int Feedback();

  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
              std::vector<std::shared_ptr<OutputDescription>> &output_descs,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois,
              std::shared_ptr<DnnNodeOutput> dnn_output);

#ifdef SHARED_MEM_ENABLED
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  int Render(const std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> &pym,
             std::string result_image,
             std::shared_ptr<HandLmkOutput> lmk_result);

  std::shared_ptr<AiMsgSubNode> ai_msg_sub_node_;
};

#endif  // MONO2D_BODY_DET_NODE_H_
