#include <algorithm>
#include <string>

#include "gtest/gtest.h"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"

TEST(Nav2Plugin, ExportedControllerCanBeLoaded)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  const auto classes = loader.getDeclaredClasses();
  const std::string expected = "path_smoothing_controller/TrajectoryPurePursuit";
  ASSERT_NE(std::find(classes.begin(), classes.end(), expected), classes.end());
  const auto controller = loader.createSharedInstance(expected);
  ASSERT_NE(controller, nullptr);
}
