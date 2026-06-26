/*
 * Copyright (c) 2026, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rqt_gui_cpp/plugin.hpp>

namespace
{

// Minimal concrete plugin used to exercise the rqt_gui_cpp::Plugin base class.
// It exposes the protected node_ member so the test can verify passInNode().
class TestPlugin
  : public rqt_gui_cpp::Plugin
{
public:
  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }
};

}  // namespace

class PluginTest
  : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(PluginTest, node_is_null_before_pass_in_node)
{
  TestPlugin plugin;
  EXPECT_EQ(plugin.node(), nullptr);
}

TEST_F(PluginTest, pass_in_node_stores_the_node)
{
  TestPlugin plugin;
  auto node = std::make_shared<rclcpp::Node>("test_rqt_gui_cpp_plugin");

  plugin.passInNode(node);

  EXPECT_EQ(plugin.node(), node);
}

TEST_F(PluginTest, shutdown_plugin_is_safe_to_call)
{
  TestPlugin plugin;
  EXPECT_NO_THROW(plugin.shutdownPlugin());
}
