#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <memory>

using boost::asio::io_service;
using boost::asio::serial_port_base;
using std::placeholders::_1;
using std::placeholders::_2;

using json = nlohmann::json;

class SerialCommunicationNode : public rclcpp::Node {
public:
    SerialCommunicationNode(std::string port, int baud_rate, int write_timmer, int read_timmer) : Node("serial_communication_node") {
        // Initialize serial port
        serial_.open(port); // Replace with your port name
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for the serial port to open
        serial_.set_option(serial_port_base::baud_rate(baud_rate));

        write_timmer_ = write_timmer;
        read_timmer_ = read_timmer;

        // Publisher for sending motor commands
        motor_commands_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_commands", 10);

        // Subscriber for receiving messages from the serial port
        serial_read_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "joy_twist_command",
            10,
            std::bind(&SerialCommunicationNode::joyTwistCommandCallback, this, _1)
        );

        // Start asynchronous communication
        sendToTeensy();
        readFromTeensy();
    }

private:
    void sendToTeensy() {
        // Send motor commands periodically
        write_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(write_timmer_), // adjust as needed
            [this]() {
                json motor_commands_json;
                motor_commands_json["vx"] = vx_;
                motor_commands_json["wz"] = wz_;

                std::string data = motor_commands_json.dump() + "\n";
                RCLCPP_INFO(this->get_logger(), "Sending data: %s", data.c_str());
                asyncWrite(data);
            }
        );
    }

    void readFromTeensy() {
        // Read messages from the serial port periodically
        read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_timmer_), // adjust as needed
            [this]() {
                asyncRead();
            }
        );
    }

    void asyncRead() {
        // Create a buffer to hold the received data
        auto read_buffer = std::make_shared<boost::asio::streambuf>();

        // Asynchronously read data from the serial port
        boost::asio::async_read_until(
            serial_,
            *read_buffer,
            '\n',
            std::bind(&SerialCommunicationNode::read_handler, this, _1, _2, read_buffer)
        );

        // Set a timer to cancel the read operation if it takes too long
        timer_ = boost::asio::steady_timer(io_);
        timer_.expires_from_now(std::chrono::milliseconds(100));
        timer_.async_wait(std::bind(&SerialCommunicationNode::timeout_handler, this, _1));

        io_.run(); // Blocks until all asynchronous operations have completed or been cancelled
        io_.reset();
    }

    void read_handler(const boost::system::error_code& ec, std::size_t bytes_transferred, std::shared_ptr<boost::asio::streambuf> read_buffer) {
        if (!ec) {
            std::istream is(read_buffer.get());
            std::string line;
            std::getline(is, line);
            RCLCPP_INFO(this->get_logger(), "Received message: %s", line.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Read error or timeout: %s", ec.message().c_str());
        }
        timer_.cancel(); // Cancel the timer as the read operation completed
    }

    void timeout_handler(const boost::system::error_code& ec) {
        if (!ec) {
            RCLCPP_INFO(this->get_logger(), "Timeout occurred.");
            serial_.cancel(); // Cancel the pending asynchronous operations
        }
    }
    
    void asyncWrite(const std::string& data) {
        boost::asio::async_write(
            serial_,
            boost::asio::buffer(data),
            [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (!ec) {
                    RCLCPP_INFO(this->get_logger(), "Data sent successfully.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Write error: %s", ec.message().c_str());
                }
            }
        );
    }

    void joyTwistCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Access linear and angular components of the twist message
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;
    }

    float vx_{0.5};
    float wz_{0.0};
    int write_timmer_{100}; 
    int read_timmer_{1000}; 
    boost::asio::io_service io_;
    boost::asio::serial_port serial_{io_};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_commands_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr serial_read_subscriber_;
    boost::asio::steady_timer timer_{io_};
    rclcpp::TimerBase::SharedPtr write_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char **argv) {
    std::string port = "/dev/ttyACM0";
    int baud_rate = 115200;
    int write_timmer = 100; // Hz
    int read_timmer = 100; // Hz
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommunicationNode>(port, baud_rate, write_timmer, read_timmer));
    rclcpp::shutdown();
    return 0;
}
