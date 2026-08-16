// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Standalone entry point. The node is also a component, so it can be loaded
// into a container alongside a camera pipeline instead; this exists for the
// common case of running one gimbal on its own.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sbgc_driver/driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sbgc_driver::SbgcDriverNode>(rclcpp::NodeOptions());

  // Single-threaded on purpose. The link has no internal locking, and while
  // every port-touching callback is already in one mutually-exclusive group,
  // there is nothing here that a second thread would make faster.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
