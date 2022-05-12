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

class LandBehaviour : public as2::BasicBehaviour<as2_msgs::action::Land> {
  public:
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;
  using PSME = as2_msgs::msg::PlatformStateMachineEvent;

  LandBehaviour()
      : as2::BasicBehaviour<as2_msgs::action::Land>(as2_names::actions::behaviours::land) {
    this->declare_parameter("default_land_plugin");
    this->declare_parameter("default_land_speed");

    loader_ = std::make_shared<pluginlib::ClassLoader<land_base::LandBase>>("land_plugin_base",
                                                                            "land_base::LandBase");

    try {
      land_speed_ =
          loader_->createSharedInstance(this->get_parameter("default_land_plugin").as_string());
      land_speed_->initialize(this);
      RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED: %s",
                  this->get_parameter("default_land_plugin").as_string().c_str());
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
    }

    // state_machine_event_cli_ = this->create_client<as2_msgs::srv::SetPlatformStateMachineEvent>(
    //     this->generate_global_name(as2_names::services::platform::set_platform_state_machine_event));
    // arming_cli_ =
    // this->create_client<std_srvs::srv::SetBool>(as2_names::services::platform::set_arming_state);
    // if (state_machine_event_cli_->wait_for_service() && arming_cli_->wait_for_service())
    // {
    //     RCLCPP_INFO(this->get_logger(), "Land Behaviour ready!");
    // }
  };

  rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const as2_msgs::action::Land::Goal> goal) {
    float land_speed = fabs(goal->land_speed);

    as2_msgs::action::Land::Goal new_goal;
    new_goal.land_speed = (land_speed != 0.0f)
                              ? -fabs(land_speed)
                              : -fabs(this->get_parameter("default_land_speed").as_double());
    auto _goal = std::make_shared<const as2_msgs::action::Land::Goal>(new_goal);

    RCLCPP_INFO(this->get_logger(), "LandBehaviour: Land with speed %f", _goal->land_speed);

    if (!this->callStateMachineServer(PSME::LAND)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
    }

    return land_speed_->onAccepted(_goal);
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleLand> goal_handle) {
    return land_speed_->onCancel(goal_handle);
  }

  void onExecute(const std::shared_ptr<GoalHandleLand> goal_handle) {
    if (land_speed_->onExecute(goal_handle)) {
      RCLCPP_INFO(this->get_logger(), "Land succeeded");

      if (!this->callStateMachineServer(PSME::LANDED)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
      }

      auto request = std_srvs::srv::SetBool::Request();
      auto response = std_srvs::srv::SetBool::Response();
      request.data = true;

      auto disarm_cli = as2::SynchronousServiceClient<std_srvs::srv::SetBool>(
          as2_names::services::platform::set_platform_state_machine_event);
      bool out = disarm_cli.sendRequest(request, response);

      if (out && response.success) {
        return;
      }

      RCLCPP_ERROR(this->get_logger(), "Unable to disarm");
    } else {
      RCLCPP_INFO(this->get_logger(), "Land canceled");
      return;
    }
  }

  private:
  bool callStateMachineServer(const int8_t machine_event) {
    auto request = as2_msgs::srv::SetPlatformStateMachineEvent::Request();
    auto response = as2_msgs::srv::SetPlatformStateMachineEvent::Response();

    request.event.event = machine_event;

    auto set_mode_cli = as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>(
        as2_names::services::platform::set_platform_state_machine_event);
    bool out = set_mode_cli.sendRequest(request, response);

    if (out && response.success) {
      return true;
    }
    return false;
  }

  private:
  std::shared_ptr<pluginlib::ClassLoader<land_base::LandBase>> loader_;
  std::shared_ptr<land_base::LandBase> land_speed_;

  // rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
  // state_machine_event_cli_; rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arming_cli_;
};

#endif  // LAND_BEHAVIOUR_HPP
