#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional> // Include for std::bind and std::placeholders

using namespace boost::asio;
using boost::asio::serial_port_base;
using json = nlohmann::json;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

io_service io;
serial_port serial(io);
steady_timer timer(io);

// Read handler adjusted for std::bind usage
void read_handler(const boost::system::error_code& ec, std::size_t bytes_transferred, std::shared_ptr<streambuf> read_buffer) {
    if (!ec) {
        std::istream is(read_buffer.get());
        std::string line;
        std::getline(is, line);
        std::cout << "Received message: " << line << std::endl;
    } else {
        std::cout << "Read error or timeout: " << ec.message() << std::endl;
    }
    timer.cancel(); // Cancel the timer as the read operation completed
}

// Write handler for async write
void write_handler(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (!ec) {
        std::cout << "Data sent successfully." << std::endl;
    } else {
        std::cout << "Write error: " << ec.message() << std::endl;
    }
}

// Timeout handler adjusted to take serial_port by reference
void timeout_handler(const boost::system::error_code& ec) {
    if (!ec) {
        std::cout << "Timeout occurred." << std::endl;
        serial.cancel(); // Cancel the pending asynchronous operations
    }
}

int main() {
    try {
        serial.open("/dev/ttyACM0"); // Replace with your port name
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for the serial port to open
        serial.set_option(serial_port_base::baud_rate(115200));

        while(true) {
            json motor_commands_json;
            motor_commands_json["vx"] = -0.1; //sin(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000.0);
            motor_commands_json["wz"] = 0.0;

            std::string data = motor_commands_json.dump() + "\n";
            std::cout << "\nNext loop" << std::endl;
            std::cout << "Sending data: " << data << std::endl;

            // Using std::bind for the write handler
            async_write(serial, buffer(data), write_handler);

            auto read_buffer = std::make_shared<streambuf>();
            // Using std::bind with placeholders for the read handler
            async_read_until(serial, *read_buffer, '\n', std::bind(read_handler, _1, _2, read_buffer));

            timer.expires_after(std::chrono::milliseconds(100)); // Set the timeout duration
            timer.async_wait(timeout_handler);

            io.run(); // Blocks until all asynchronous operations have completed or been cancelled
            io.reset(); // Reset the io_service to run it again for the next loop
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
