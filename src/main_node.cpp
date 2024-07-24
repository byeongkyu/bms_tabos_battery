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
            this->declare_parameter<int>("batt_cnt", 5);


            auto interface_name = this->get_parameter("interface_name").get_parameter_value().get<std::string>();
            prefix_ = this->get_parameter("prefix").get_parameter_value().get<std::string>();
            batt_cnt_ = this->get_parameter("batt_cnt").get_parameter_value().get<uint8_t>();
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
            tv.tv_usec = 10000;  // 10ms
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

            batt_msgs_.resize(batt_cnt_, sensor_msgs::msg::BatteryState());
            for(uint8_t i = 0; i < batt_cnt_; i++)
            {
                char topic_name[100];
                snprintf(topic_name, 100, "batt%d/battery_state", (int)i);
                pub_batt_states_.push_back(this->create_publisher<sensor_msgs::msg::BatteryState>(topic_name, rclcpp::SystemDefaultsQoS()));
            }

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&BMSTabosBatteryNode::timer_callback, this));
        }
        ~BMSTabosBatteryNode() {}

    private:
        void timer_callback()
        {
            struct can_frame send_frame;
            struct can_frame recv_frame;

            for(uint8_t id = 1; id <= batt_cnt_; id++)
            {
                send_frame.can_id = 0x460 + id;
                send_frame.can_dlc = 1;
                send_frame.data[0] = 0x60 + id;

                if (::write(can_sock_, &send_frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
                {
                    RCLCPP_ERROR(this->get_logger(), "SocketCAN Write Error");
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));

                while (rclcpp::ok())
                {
                    auto ret = ::read(can_sock_, &recv_frame, sizeof(struct can_frame));
                    if (ret <= 0)
                    {
                        // RCLCPP_ERROR(this->get_logger(), "SocketCAN Read Error");
                        break;
                    }

                    batt_msgs_[id - 1].header.stamp = this->now();

                    char location_str[100];
                    snprintf(location_str, 100, "SLOT%d", id);
                    batt_msgs_[id - 1].location = location_str;
                    batt_msgs_[id - 1].present = true;
                    batt_msgs_[id - 1].power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

                    if(recv_frame.data[0] == (0x60 + id) && recv_frame.data[1] == 1)
                    {
                        batt_msgs_[id - 1].voltage = (uint16_t)((recv_frame.data[3] << 8) + recv_frame.data[2]) / 100.0;
                        batt_msgs_[id - 1].current = (int16_t)((recv_frame.data[5] << 8) + recv_frame.data[4]) / 100.0;

                        RCLCPP_DEBUG(this->get_logger(), "%f", batt_msgs_[id - 1].voltage);

                        if(batt_msgs_[id - 1].current > 0)
                        {
                            batt_msgs_[id - 1].power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
                            batt_msgs_[id - 1].charge = (int16_t)((recv_frame.data[5] << 8) + recv_frame.data[4]) / 100.0;
                        }
                        else if(batt_msgs_[id - 1].current == 0)
                        {
                            batt_msgs_[id - 1].power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
                            batt_msgs_[id - 1].charge = 0.0;
                        }
                        else
                        {
                            batt_msgs_[id - 1].power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                            batt_msgs_[id - 1].charge = 0.0;
                        }

                        uint16_t status = (uint16_t)((recv_frame.data[7] << 8) + recv_frame.data[6]);
                        if(status & 0x01)
                        {
                            batt_msgs_[id - 1].power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
                        }
                        else if(status & 0x10)
                        {
                            batt_msgs_[id - 1].power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
                        }
                        else if(status & 0x20)
                        {
                            batt_msgs_[id - 1].power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD;
                        }
                        else
                        {
                            batt_msgs_[id - 1].power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
                        }
                    }
                    else if(recv_frame.data[0] == (0x60 + id) && recv_frame.data[1] == 2)
                    {
                        batt_msgs_[id - 1].percentage = (uint8_t)recv_frame.data[6];
                        uint8_t soh = (uint8_t)recv_frame.data[7];
                        auto ttf = (uint16_t)((recv_frame.data[3] << 8) + recv_frame.data[2]);
                        auto tte = (uint16_t)((recv_frame.data[5] << 8) + recv_frame.data[4]);
                    }
                    else if(recv_frame.data[0] == (0x60 + id) && recv_frame.data[1] == 3)
                    {
                        batt_msgs_[id - 1].capacity = (uint16_t)((recv_frame.data[3] << 8) + recv_frame.data[2]) / 100.0;
                        batt_msgs_[id - 1].temperature = (int16_t)((recv_frame.data[7] << 8) + recv_frame.data[6]) / 10.0;
                    }

                    RCLCPP_DEBUG(this->get_logger(), "RECV DATA = ID: [%3X] LEN: [%d] DATA: [%2X %2X %2X %2X %2X %2X %2X %2X]",
                        recv_frame.can_id, recv_frame.can_dlc,
                        recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3],
                        recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]
                    );
                }

                pub_batt_states_[id - 1]->publish(batt_msgs_[id - 1]);
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<sensor_msgs::msg::BatteryState> batt_msgs_;
        std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>>> pub_batt_states_;

        int can_sock_;
        std::string prefix_;
        uint8_t batt_cnt_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMSTabosBatteryNode>());
    rclcpp::shutdown();
    return 0;
}