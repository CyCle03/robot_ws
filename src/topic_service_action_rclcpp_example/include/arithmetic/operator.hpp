#ifndef ARITHMETIC__OPERATOR_HPP_
#define ARITHMETIC__OPERATOR_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"


class Operator : public rclcpp::Node
{
public:
  using ArithmeticOperator = msg_srv_action_interface_example::srv::ArithmeticOperator;

  explicit Operator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Operator();

  void send_request();

private:
  rclcpp::Client<ArithmeticOperator>::SharedPtr arithmetic_service_client_;
};
#endif  // ARITHMETIC__OPERATOR_HPP_
