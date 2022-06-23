#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

class ServiceServerTest : public rclcpp::Node {
 public:
  ServiceServerTest(const rclcpp::NodeOptions& options) : rclcpp::Node("service_server", options) {
    service_name_ = declare_parameter("service_name", std::string{});
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
        service_name_, std::bind(&ServiceServerTest::add, this, _1, _2));
  }

  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO_STREAM(get_logger(), "Server " << service_name_ << " Responding");
  }

 private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  std::string service_name_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto fixture = std::make_shared<ServiceServerTest>(rclcpp::NodeOptions());
  executor.add_node(fixture->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
