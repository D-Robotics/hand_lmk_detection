// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <string>

#include "include/ai_msg_sub_node.h"
#include "include/hand_lmk_det_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("hand lmk det pkg"),
              "This is hand lmk det pkg!");

  rclcpp::ExecutorOptions options = rclcpp::ExecutorOptions();
  rclcpp::executors::MultiThreadedExecutor mult_exe(
      options, 3, false, std::chrono::milliseconds(10));
  auto ai_sub_node = std::make_shared<AiMsgSubNode>("ai_sub_det");
  mult_exe.add_node(ai_sub_node);
  auto det_node = std::make_shared<HandLmkDetNode>("hand_lmk_det", ai_sub_node);
  mult_exe.add_node(det_node);
  mult_exe.spin();

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("hand lmk det pkg"), "Pkg exit.");
  return 0;
}
