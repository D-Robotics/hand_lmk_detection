// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/ai_msg_sub_node.h"

#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

AiMsgSubNode::AiMsgSubNode(const std::string& node_name,
                           const NodeOptions& options)
    : rclcpp::Node(node_name, options) {
  this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                       ai_msg_sub_topic_name_);
  this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                   ai_msg_sub_topic_name_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n ai_msg_sub_topic_name_: " << ai_msg_sub_topic_name_;
  RCLCPP_WARN(
      rclcpp::get_logger("hand lmk ai msg sub"), "%s", ss.str().c_str());

  RCLCPP_WARN(rclcpp::get_logger("hand lmk ai msg sub"),
              "Create subscription with topic_name: %s",
              ai_msg_sub_topic_name_.c_str());
  ai_msg_subscription_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
          ai_msg_sub_topic_name_,
          10,
          std::bind(&AiMsgSubNode::AiMsgProcess, this, std::placeholders::_1));
}

AiMsgSubNode::~AiMsgSubNode() {}

void AiMsgSubNode::AiMsgProcess(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved ai msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_"
     << msg->header.stamp.nanosec;
  RCLCPP_INFO(
      rclcpp::get_logger("hand lmk ai msg sub"), "%s", ss.str().c_str());

  hand_lmk_feed_cache_.Feed(msg);
}

int AiMsgSubNode::GetTargetRois(
    const std_msgs::msg::Header::_stamp_type& msg_ts,
    std::shared_ptr<std::vector<hbDNNRoi>>& rois,
    std::map<size_t, size_t>& valid_roi_idx,
    ai_msgs::msg::PerceptionTargets::UniquePtr& ai_msg,
    int time_out_ms) {
  std::string ts =
      std::to_string(msg_ts.sec) + "." + std::to_string(msg_ts.nanosec);
  ai_msg = hand_lmk_feed_cache_.Get(msg_ts, time_out_ms);
  if (!ai_msg) {
    RCLCPP_WARN(rclcpp::get_logger("hand lmk ai msg sub"),
                "Frame find ai ts %s fail",
                ts.c_str());
    return -1;
  }

  if (ai_msg->targets.empty()) {
    return 0;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
               "Frame ai ts: %s targets size: %d",
               ts.c_str(),
               ai_msg->targets.size());
  size_t hand_roi_idx = 0;
  for (const auto target : ai_msg->targets) {
    RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                 "target.rois.size: %d",
                 target.rois.size());
    for (const auto& roi : target.rois) {
      RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                   "roi.type: %s",
                   roi.type.c_str());
      if ("hand" == roi.type) {
        RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                     "recv roi x_offset: %d y_offset: %d width: %d height: %d",
                     roi.rect.x_offset,
                     roi.rect.y_offset,
                     roi.rect.width,
                     roi.rect.height);

        int32_t left = roi.rect.x_offset;
        int32_t top = roi.rect.y_offset;
        int32_t right = roi.rect.x_offset + roi.rect.width;
        int32_t bottom = roi.rect.y_offset + roi.rect.height;

        // roi's left and top must be even, right and bottom must be odd
        left += (left % 2 == 0 ? 0 : 1);
        top += (top % 2 == 0 ? 0 : 1);
        right -= (right % 2 == 1 ? 0 : 1);
        bottom -= (bottom % 2 == 1 ? 0 : 1);

        RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                     "roi: %d %d %d %d",
                     left,
                     top,
                     right,
                     bottom);

        int32_t roi_w = right - left;
        int32_t roi_h = bottom - top;
        int32_t max_size = std::max(roi_w, roi_h);
        int32_t min_size = std::min(roi_w, roi_h);

        if (max_size < roi_size_max_ && min_size > roi_size_min_) {
          if (!rois) {
            rois = std::make_shared<std::vector<hbDNNRoi>>();
          }

          rois->push_back({left, top, right, bottom});
          RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                       "rois size: %d",
                       rois->size());
          // 原始roi的索引对应于valid_rois的索引
          valid_roi_idx[hand_roi_idx] = rois->size() - 1;

          RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                       "Valid hand roi map: %d %d",
                       hand_roi_idx,
                       valid_roi_idx[hand_roi_idx]);

          RCLCPP_DEBUG(rclcpp::get_logger("hand lmk ai msg sub"),
                       "Valid hand roi: %d %d %d %d, roi_w: %d, roi_h: %d, "
                       "max_size: %d, min_size: %d",
                       left,
                       top,
                       right,
                       bottom,
                       roi_w,
                       roi_h,
                       max_size,
                       min_size);
        } else {
          RCLCPP_WARN(
              rclcpp::get_logger("hand lmk ai msg sub"),
              "Filter hand roi: %d %d %d %d, max_size: %d, min_size: %d",
              left,
              top,
              right,
              bottom,
              max_size,
              min_size);
        }

        hand_roi_idx++;
      }
    }
  }
  return 0;
}
