#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "calculator/calculator.hpp"


void print_help()
{
  printf("For ROS 2 topic subscriber, service server, action server rclcpp examples:\n");
  printf("calculator [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}


int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto calculator = std::make_shared<Calculator>();

  rclcpp::spin(calculator);

  rclcpp::shutdown();

  return 0;
}
