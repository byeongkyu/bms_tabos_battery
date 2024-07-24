#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/battery_state.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>


class BMSTabosBatteryNode: public rclcpp::Node
{
    public:
        BMSTabosBatteryNode(): Node("bms_tabos_battery_node")
        {
            this->declare_parameter<std::string>("interface_name", "can0");
            this->declare_parameter<double>("rate", 5.0);
            this->declare_parameter<std::string>("prefix", "");

            auto interface_name = this->get_parameter("interface_name").get_parameter_value().get<std::string>();
            prefix_ = this->get_parameter("prefix").get_parameter_value().get<std::string>();
            RCLCPP_INFO(this->get_logger(), "Connect BMS via interface: [%s] and prefix: [%s]", interface_name.c_str(), prefix_.c_str());

            // SocketCAN Initialize
            struct ifreq ifr;
            struct sockaddr_can addr;

            memset(&ifr, 0, sizeof(ifr));
            memset(&addr, 0, sizeof(addr));

            can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            strcpy(ifr.ifr_name, interface_name.c_str());
            ioctl(can_sock_, SIOCGIFINDEX, &ifr);

            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 500000;  // 500ms
            setsockopt(can_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            addr.can_ifindex = ifr.ifr_ifindex;
            addr.can_family = PF_CAN;

            if (bind(can_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "SocketCAN Bind Error.");
                assert(true);
            }
            RCLCPP_INFO(this->get_logger(), "SocketCAN Initialized.");

            // ROS Related
            pub_batt_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SystemDefaultsQoS());

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&BMSTabosBatteryNode::timer_callback, this));
        }
        ~BMSTabosBatteryNode() {}

    private:
        void timer_callback()
        {
            struct can_frame send_frame;
            struct can_frame recv_frame;

            send_frame.can_id = 0x460 + 1;
            send_frame.can_dlc = 1;
            send_frame.data[0] = 0x60;

            if (::write(can_sock_, &send_frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
            {
                RCLCPP_ERROR(this->get_logger(), "SocketCAN Write Error");
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            while (rclcpp::ok())
            {
                auto ret = ::read(can_sock_, &recv_frame, sizeof(struct can_frame));
                if (ret < 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "SocketCAN Read Error");
                    return;
                }
                else if(ret == 0)
                {
                    return;
                }

                RCLCPP_ERROR(this->get_logger(), "RECV DATA = ID: [%3X] LEN: [%d] DATA: [%2X %2X %2X %2X %2X %2X %2X %2X",
                    recv_frame.can_id, recv_frame.can_dlc,
                    recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3],
                    recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]
                );
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> pub_batt_state_;

        int can_sock_;
        std::string prefix_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMSTabosBatteryNode>());
    rclcpp::shutdown();
    return 0;
}