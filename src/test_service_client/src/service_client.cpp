#include <memory>
#include <thread>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class ServiceClientTest : public rclcpp::Node {
 public:
  ServiceClientTest(const rclcpp::NodeOptions& options) : rclcpp::Node("service_client", options) {
    auto servers = declare_parameter("servers_names", std::vector<std::string>{});
    for (auto& server_name : servers) {
      // Adding one callback per client, or one callback for all clients, doesn't solve
      // auto callback_group_client = create_callback_group(
      //     rclcpp::CallbackGroupType::MutuallyExclusive);  // Using Reentrant doesn't help
      // callback_groups_clients_.push_back(callback_group_client);
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
          create_client<example_interfaces::srv::AddTwoInts>(
              server_name, rmw_qos_profile_services_default);  //, callback_group_client);
      clients_.push_back(client);
    }
    node_test_thread_ = std::make_shared<std::thread>(&ServiceClientTest::callThread, this);
  }

  ~ServiceClientTest() {
    if (node_test_thread_->joinable()) node_test_thread_->join();
  }

  // Using a timer instead of a thread doesn't help.
  void callThread() {
    while (rclcpp::ok()) {
      for (auto& client : clients_) {
        if (!client->wait_for_service(3s))
          RCLCPP_ERROR_STREAM(get_logger(), "Waiting service failed");  // Never happens
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        RCLCPP_INFO_STREAM(get_logger(), "Calling service " << client->get_service_name());
        auto result = client->async_send_request(request);  // Specifying a callback doesn't help
        if (result.wait_for(3s) != std::future_status::ready) {  // More time doesn't help
          RCLCPP_ERROR_STREAM(
              get_logger(), "Waiting result failed for server " << client->get_service_name());
          return;
        } else {
          RCLCPP_INFO_STREAM(
              get_logger(), "Waiting succeeded for server " << client->get_service_name());
        }
      }
      // std::this_thread::sleep_for(100ms);  // On Galactic or earlier
      get_clock()->sleep_for(rclcpp::Duration(100ms));  // On Humble only
      // Looping faster (ex 10ms) helps reproducing the issue more frequently
    }
  }

 private:
  std::vector<rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr> clients_;
  std::vector<rclcpp::CallbackGroup::SharedPtr>
      callback_groups_clients_;  // Using only one cb group for all the clients also doesn't help
  std::shared_ptr<std::thread> node_test_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // The more threads are assigned and the less likely it is to get a deadlock if using multiple
  // callback groups. Confirmed to happen with 5 threads.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  auto test_node = std::make_shared<ServiceClientTest>(rclcpp::NodeOptions());
  executor.add_node(test_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
