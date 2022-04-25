#ifndef LAND_BASE_HPP
#define LAND_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include <as2_msgs/action/land.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace land_base
{
    class LandBase
    {
    public:
        using GoalHandleLand = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;

        void initialize(as2::Node *node_ptr)
        {
            node_ptr_ = node_ptr;
            odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
                node_ptr_->generate_global_name(as2_names::topics::self_localization::odom),
                as2_names::topics::self_localization::qos,
                std::bind(&LandBase::odomCb, this, std::placeholders::_1));

            this->ownInit(node_ptr_);
        };

        virtual rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::Land::Goal> goal) = 0;
        virtual rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleLand> goal_handle) = 0;
        virtual bool onExecute(const std::shared_ptr<GoalHandleLand> goal_handle) = 0;

        virtual ~LandBase(){};

    protected:
        LandBase(){};

        // To initialize needed publisher for each plugin
        virtual void ownInit(as2::Node *node_ptr){};

    private:
        // TODO: if onExecute is done with timer no atomic attributes needed
        void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
            this->actual_heigth_ = msg->pose.pose.position.z;
            this->actual_z_speed_ = msg->twist.twist.linear.z;
        };

    protected:
        as2::Node *node_ptr_;

        std::atomic<float> actual_heigth_;
        std::atomic<float> actual_z_speed_;

        float desired_speed_ = 0.0;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    }; // LandBase class

} // land_base namespace

#endif // LAND_BASE_HPP