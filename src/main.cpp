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
