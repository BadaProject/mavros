/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ActuatorOutputs plugin
 * @file actuator_outputs.cpp
 * @author Jeyong Shin <jeyong@subak.io>
 * 
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/actuator_outputs.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief ActuatorControl plugin
 * @plugin actuator_control
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorOutputsPlugin : public plugin::Plugin
{
public:
  explicit ActuatorOutputsPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "actuator_outputs")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    actuator_outputs_pub = node->create_publisher<mavros_msgs::msg::ActuatorOutputs>(
      "target_actuator_outputs", sensor_qos);
    actuator_outputs_sub = node->create_subscription<mavros_msgs::msg::ActuatorOutputs>(
      "actuator_outputs", sensor_qos, std::bind(
        &ActuatorOutputsPlugin::actuator_outputs_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ActuatorOutputsPlugin::handle_actuator_control_target),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_outputs_pub;
  rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_outputs_sub;

  /* -*- rx handlers -*- */

  void handle_actuator_control_target(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ACTUATOR_CONTROL_TARGET & act,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    auto ract = mavros_msgs::msg::ActuatorControl();
    ract.header.stamp = uas->synchronise_stamp(act.time_usec);
    ract.group_mix = act.group_mlx;
    ract.controls = act.controls;

    actuator_outputs_pub->publish(ract);
  }

  /* -*- callbacks -*- */

  void actuator_outputs_cb(const mavros_msgs::msg::ActuatorOutputs::SharedPtr req)
  {
    //! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
    //! message definiton here: @p https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_STATUS
    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS aout{};
    act.time_usec = get_time_usec(req->header.stamp);
    uas->msg_set_target(act);
    act.controls = req->outputs;

    uas->send_message(act);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ActuatorOutputsPlugin)
