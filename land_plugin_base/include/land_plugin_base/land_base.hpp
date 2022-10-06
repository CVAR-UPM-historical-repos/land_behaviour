/*!*******************************************************************************************
 *  \file       land_base.hpp
 *  \brief      land_base header file
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

#ifndef LAND_BASE_HPP
#define LAND_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <as2_msgs/action/land.hpp>

namespace land_base
{
    class LandBase
    {
    public:
        using GoalHandleLand = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;

        void initialize(as2::Node *node_ptr)
        {
            node_ptr_ = node_ptr;

            pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(node_ptr_, as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
            twist_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(node_ptr_, as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
            synchronizer_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(5), *(pose_sub_.get()), *(twist_sub_.get()));
            synchronizer_->registerCallback(&LandBase::state_callback, this);

            node_ptr_->declare_parameter<std::string>("frame_id_pose", "");
            node_ptr_->get_parameter("frame_id_pose", frame_id_pose_);

            node_ptr_->declare_parameter<std::string>("frame_id_twist", "");
            node_ptr_->get_parameter("frame_id_twist", frame_id_twist_);

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
        void state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                            const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
        {
            this->actual_heigth_ = pose_msg->pose.position.z;
            this->actual_z_speed_ = twist_msg->twist.linear.z;
            return;
        };

    protected:
        as2::Node *node_ptr_;

        std::atomic<float> actual_heigth_;
        std::atomic<float> actual_z_speed_;

        float desired_speed_ = 0.0;

        std::string frame_id_pose_ = "";
        std::string frame_id_twist_ = "";

    private:
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> twist_sub_;
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> approximate_policy;
        std::shared_ptr<message_filters::Synchronizer<approximate_policy>> synchronizer_;
    }; // LandBase class

} // land_base namespace

#endif // LAND_BASE_HPP