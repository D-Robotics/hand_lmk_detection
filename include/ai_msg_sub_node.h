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
#include <utility>
#include <queue>
#include <unordered_map>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"
#include "rclcpp/rclcpp.hpp"

#ifndef AI_MSG_SUB_NODE_H_
#define AI_MSG_SUB_NODE_H_

using ai_msgs::msg::PerceptionTargets;
using rclcpp::NodeOptions;

using feed_predict_type =
    std::shared_ptr<std::pair<ai_msgs::msg::PerceptionTargets::ConstSharedPtr,
                              sensor_msgs::msg::Image::ConstSharedPtr>>;
class HandLmkFeedCache {
 public:
  explicit HandLmkFeedCache(int cache_lint_len = 10) :
  cache_lint_len_(cache_lint_len) {}
  ~HandLmkFeedCache() {}

  int Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg) {
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(
        new ai_msgs::msg::PerceptionTargets());
    ai_msg->set__header(msg->header);
    ai_msg->set__fps(msg->fps);
    ai_msg->set__targets(msg->targets);
    ai_msg->set__disappeared_targets(msg->disappeared_targets);
    ai_msg->set__perfs(msg->perfs);
    std::string ts = std::to_string(msg->header.stamp.sec) + "." +
                     std::to_string(msg->header.stamp.nanosec);
    RCLCPP_DEBUG(rclcpp::get_logger("example"), "Feed ts %s", ts.c_str());

    std::unique_lock<std::mutex> lg(cache_mtx_);
    // 判断是否超过最大长度
    if (recved_input_cache_.size() > cache_lint_len_) {
      recved_input_cache_.erase(recved_input_cache_.begin());
    }
    recved_aimsg_cache_[ts] = std::move(ai_msg);
    return 0;
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr Get(const std::string& ts,
                                                 int time_out_ms = 1000) {
    RCLCPP_DEBUG(rclcpp::get_logger("example"), "Get ts %s", ts.c_str());

    ai_msgs::msg::PerceptionTargets::UniquePtr feed_predict = nullptr;
    std::unique_lock<std::mutex> lg(cache_mtx_);
    cache_cv_.wait_for(lg, std::chrono::milliseconds(time_out_ms), [&]() {
      return recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end();
    });
    if (recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end()) {
      feed_predict = std::move(recved_aimsg_cache_.at(ts));
      recved_aimsg_cache_.erase(ts);
    }

    return feed_predict;
  }

 private:
  std::queue<feed_predict_type> feed_predict_cache_;
  // key is ts
  std::unordered_map<std::string, feed_predict_type> recved_input_cache_;
  size_t cache_lint_len_ = 10;
  std::mutex cache_mtx_;
  std::condition_variable cache_cv_;
  std::unordered_map<std::string, ai_msgs::msg::PerceptionTargets::UniquePtr>
      recved_aimsg_cache_;
};

class AiMsgSubNode : public rclcpp::Node {
 public:
  AiMsgSubNode(const std::string& node_name,
               const NodeOptions& options = NodeOptions());
  ~AiMsgSubNode() override;

  int GetTargetRois(const std::string& ts,
                    std::shared_ptr<std::vector<hbDNNRoi>>& rois,
                    std::map<size_t, size_t>& valid_roi_idx,
                    ai_msgs::msg::PerceptionTargets::UniquePtr& ai_msg,
                    int time_out_ms = 200);

 private:
  HandLmkFeedCache hand_lmk_feed_cache_;
  // resizer model input size limit
  // roi, width & hight must be in range [16, 256)
  int32_t roi_size_max_ = 255;
  int32_t roi_size_min_ = 16;

  std::string ai_msg_sub_topic_name_ = "/hobot_mono2d_body_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      ai_msg_subscription_ = nullptr;
  void AiImgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
};

#endif  // AI_MSG_SUB_NODE_H_
