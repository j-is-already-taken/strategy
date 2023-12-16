#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int openSerial(const char *device_name) {
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }

    fcntl(fd, F_SETFL, 0);

    // Load configuration
    struct termios conf_tio;
    tcgetattr(fd, &conf_tio);

    // Set baudrate
    speed_t baudrate = B9600;
    cfsetispeed(&conf_tio, baudrate);
    cfsetospeed(&conf_tio, baudrate);

    // Non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);

    // Non blocking
    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    // Store configuration
    tcsetattr(fd, TCSANOW, &conf_tio);

    return fd;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("s4_comport_serialport");
    auto serial_pub = node->create_publisher<std_msgs::msg::String>("Serial_in", 1000);

    char device_name[] = "/dev/ttyACM0";
    int fd = openSerial(device_name);

    if (fd < 0) {
        RCLCPP_ERROR(node->get_logger(), "Serial Fail: could not open %s", device_name);
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::WallRate loop_rate(200);

    while (rclcpp::ok()) {
        char buf[256] = {0};
        std::string data;
        int len = 0;
        int flag = 0;

        while (true) {
            int recv_data = read(fd, buf, sizeof(buf));

            if (recv_data > 0) {
                flag = 1;
                len += recv_data;
                data += std::string(buf, recv_data);

                if (data.back() == '\n') {
                    if (len > 1) {
                        auto serial_msg = std::make_unique<std_msgs::msg::String>();
                        serial_msg->data = data;
                        serial_pub->publish(std::move(serial_msg));
                    }
                    break;
                }
            } else {
                if (flag == 0) break;
            }
        }
        loop_rate.sleep();
    } 

    rclcpp::shutdown();
    return 0;
}
