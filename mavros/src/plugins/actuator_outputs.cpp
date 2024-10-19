/*
 * Copyright 2015 Andreas Antener <andreas@uaventure.com>.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ActuatorOutputsPlugin plugin
 * @file actuator_outputs.cpp
 * @author Jeyong Shin <jeyong@subak.io>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/actuator_outputs.hpp"

namespace mavros
{
namespace std_plugins
{

/**
 * @brief Altitude plugin.
 * @plugin altitude
 */
class ActuatorOutputsPlugin : public plugin::Plugin
{
public:
  explicit ActuatorOutputsPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "actuator_outputs")
  {

    auto sensor_qos = rclcpp::SensorDataQoS();

    actuator_outputs_pub = node->create_publisher<mavros_msgs::msg::ActuatorOutputs>("actuator_outputs", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ActuatorOutputsPlugin::handle_actuator_outputs),
    };
  }

private:
  std::string frame_id;

  rclcpp::Publisher<mavros_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_pub;

  void handle_actuator_outputs(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS & actuator_outputs, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ros_msg = mavros_msgs::msg::ActuatorOutputs();
    ros_msg.active = actuator_outputs.active;
    ros_msg.actuator = actuator_outputs.actuator;

    actuator_outputs_pub->publish(ros_msg);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ActuatorOutputsPlugin)
