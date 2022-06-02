/*!*******************************************************************************************
 *  \file       land_behaviour.hpp
 *  \brief      land_behaviour header file
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef LAND_BEHAVIOUR_HPP
#define LAND_BEHAVIOUR_HPP

#include <as2_msgs/action/land.hpp>
#include <chrono>
#include <pluginlib/class_loader.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "land_plugin_base/land_base.hpp"

class LandBehaviour : public as2::BasicBehaviour<as2_msgs::action::Land>
{
public:
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;
  using PSME = as2_msgs::msg::PlatformStateMachineEvent;

  LandBehaviour()
      : as2::BasicBehaviour<as2_msgs::action::Land>(as2_names::actions::behaviours::land)
  {
    try
    {
      // this->declare_parameter<std::string>("default_land_plugin");
      this->declare_parameter("default_land_plugin"); // TODO: Fix for ROS2 Galactic
    }
    catch(const rclcpp::ParameterTypeException& e)
    {
      RCLCPP_FATAL(this->get_logger(), "Launch argument <default_land_plugin> not defined or malformed: %s", e.what());
      this->~LandBehaviour();
    }
    try
    {
      // this->declare_parameter<double>("default_land_speed");
      this->declare_parameter("default_land_speed"); // TODO: Fix for ROS2 Galactic
    }
    catch(const rclcpp::ParameterTypeException& e)
    {
      RCLCPP_FATAL(this->get_logger(), "Launch argument <default_land_speed> not defined or malformed: %s", e.what());
      this->~LandBehaviour();
    }
    
    loader_ = std::make_shared<pluginlib::ClassLoader<land_base::LandBase>>("land_plugin_base",
                                                                            "land_base::LandBase");

    try
    {
      std::string plugin_name = this->get_parameter("default_land_plugin").as_string();
      plugin_name += "::Plugin";
      land_plugin_ = loader_->createSharedInstance(plugin_name);
      land_plugin_->initialize(this);
      RCLCPP_INFO(this->get_logger(), "LAND PLUGIN LOADED: %s", plugin_name.c_str());
    }
    catch (pluginlib::PluginlibException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
    }
  };

  ~LandBehaviour(){};

  rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const as2_msgs::action::Land::Goal> goal)
  {
    float land_speed = fabs(goal->land_speed);

    as2_msgs::action::Land::Goal new_goal;
    new_goal.land_speed = (land_speed != 0.0f)
                              ? -fabs(land_speed)
                              : -fabs(this->get_parameter("default_land_speed").as_double());
    auto _goal = std::make_shared<const as2_msgs::action::Land::Goal>(new_goal);

    RCLCPP_INFO(this->get_logger(), "LandBehaviour: Land with speed %f", _goal->land_speed);

    if (!this->callStateMachineServer(PSME::LAND))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
    }

    return land_plugin_->onAccepted(_goal);
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    return land_plugin_->onCancel(goal_handle);
  }

  void onExecute(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    if (land_plugin_->onExecute(goal_handle))
    {
      RCLCPP_INFO(this->get_logger(), "Land succeeded");

      if (!this->callStateMachineServer(PSME::LANDED))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
      }

      auto request = std_srvs::srv::SetBool::Request();
      auto response = std_srvs::srv::SetBool::Response();
      request.data = true;

      auto disarm_cli = as2::SynchronousServiceClient<std_srvs::srv::SetBool>(
          as2_names::services::platform::set_arming_state);
      bool out = disarm_cli.sendRequest(request, response);

      if (out && response.success)
      {
        return;
      }

      RCLCPP_ERROR(this->get_logger(), "Unable to disarm");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Land canceled");
      return;
    }
  }

private:
  bool callStateMachineServer(const int8_t machine_event)
  {
    auto request = as2_msgs::srv::SetPlatformStateMachineEvent::Request();
    auto response = as2_msgs::srv::SetPlatformStateMachineEvent::Response();

    request.event.event = machine_event;

    auto set_mode_cli = as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>(
        as2_names::services::platform::set_platform_state_machine_event);
    bool out = set_mode_cli.sendRequest(request, response);

    if (out && response.success)
    {
      return true;
    }
    return false;
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<land_base::LandBase>> loader_;
  std::shared_ptr<land_base::LandBase> land_plugin_;
};

#endif // LAND_BEHAVIOUR_HPP
