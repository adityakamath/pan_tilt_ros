#include <cmath>
#include <SCServo_Linux/SCServo.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

using namespace std;

class PanTiltCtrlNode : public rclcpp::Node
{
public:
    PanTiltCtrlNode() : Node("pan_tilt_ctrl_node")
    {
        speed_        = this->declare_parameter<int>("speed", 4500); // steps per second
        acceleration_ = this->declare_parameter<int>("acceleration", 255); // 0 - 255
        stop_button_  = this->declare_parameter<int>("stop_button", 4); // defaults to SixAxis/SteamDeck:L1
        joint_ids_    = this->declare_parameter<vector<long int>>("joint_ids", {1, 1});
        
        string usb_port = this->declare_parameter<string>("usb_port", "/dev/ttyACM0");
        int baud_rate   = this->declare_parameter<int>("baud_rate", 1000000); //115200 for sms, 1000000 for sts

        if (joint_ids_.size() != 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameters. 2 joint names, IDs and angles are required. Currently only two motor (pan-tilt) configurations are supported");
            return;
        }

        if(!st3215_.begin(baud_rate, usb_port.c_str()))
        {
            stop_ = true;
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize motors");
            return;
        }

        for (size_t i = 0; i < joint_ids_.size(); ++i)
        {
            stop_ = false;
            st3215_.Mode(joint_ids_[i], 0); // set to mode 0 - closed loop servo mode
        }

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&PanTiltCtrlNode::JoyCallback, this, placeholders::_1));
        joint_cmd_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_commands", 10, bind(&PanTiltCtrlNode::JointCmdCallback, this, placeholders::_1));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&PanTiltCtrlNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Initialized");
    }

    ~PanTiltCtrlNode()
    {
        for (size_t i = 0; i < joint_ids_.size(); ++i)
        {
            stop_ = true;
            st3215_.EnableTorque(static_cast<uint8_t>(joint_ids_[i]), 0);
        }
        st3215_.end();
        RCLCPP_INFO(this->get_logger(), "Motors Disabled");
    }
private:
    SMS_STS st3215_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr joy_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_cmd_msg_;

    vector<long int> joint_ids_;
    int speed_;
    int acceleration_;
    int stop_button_;
    bool stop_;

    unordered_map<string, long int> joint_id_map_;

    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if ( msg->buttons[stop_button_] && !joy_msg_->buttons[stop_button_] )
        {
            stop_ = !(stop_);
        }

        joy_msg_ = msg;
    }

    void JointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_cmd_msg_ = msg;
    }

    void TimerCallback()
    {
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        auto diagnostic_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
        
        joint_state_msg->header.stamp = this->now();
        
        if(joint_cmd_msg_ && joint_cmd_msg_->name.size() == joint_ids_.size())
        {
            for(size_t i = 0; i < joint_cmd_msg_->name.size(); ++i)
            {
                string j = joint_cmd_msg_->name[i];
                uint8_t u8_id = static_cast<uint8_t>(joint_ids_[i]);

                // write to servos
                if (!stop_)
                {
                    st3215_.RegWritePosEx(u8_id, 
                                          static_cast<int>(round(joint_cmd_msg_->position[i]*(4096.0/(2*M_PI)))), 
                                          speed_, 
                                          acceleration_);
                    st3215_.RegWriteAction();
                }
                else
                {
                    st3215_.EnableTorque(u8_id, 0);
                }

                // read feedback data
                double position = 0.0, speed = 0.0, pwm = 0.0, temperature = 0.0, voltage = 0.0, current = 0.0;
                int move = 0;
            
                if ( st3215_.FeedBack(u8_id) != -1 )
                {
                    position = 2*M_PI - st3215_.ReadPos(u8_id)*2*M_PI/4096.0; // 1 step=2*PI/4096.0 rad, 
                    speed = -1 * st3215_.ReadSpeed(u8_id)*2*M_PI/4096.0;  // 1 steps/s=2*PI/4096.0 rads/sec
                    pwm = -1 * st3215_.ReadLoad(u8_id)/10.0; // 0-1000 : 0-100%
                    move = st3215_.ReadMove(u8_id);
                    temperature = st3215_.ReadTemper(u8_id); // 1 : 1 degree Celcius
                    voltage = st3215_.ReadVoltage(u8_id)/10; // 1 : 0.1 V
                    current = st3215_.ReadCurrent(u8_id)*6.5/1000; // 1 : 6.5/1000 A
                }

                // populate joint state message
                joint_state_msg->name.push_back(joint_cmd_msg_->name[i]);
                joint_state_msg->position.push_back(position);
                joint_state_msg->velocity.push_back(speed);
                joint_state_msg->effort.push_back(current);

                // populate diagnostics message
                diagnostic_msg->status.emplace_back();
                diagnostic_msg->status[i].name = j;
                diagnostic_msg->status[i].hardware_id = to_string(u8_id);
                diagnostic_msg->status[i].message = j;

                diagnostic_msgs::msg::KeyValue pwm_msg, move_msg, temp_msg, voltage_msg, current_msg;

                pwm_msg.key = "pwm";
                pwm_msg.value = to_string(pwm);

                move_msg.key = "move";
                move_msg.value = to_string(move);

                temp_msg.key = "temperature";
                temp_msg.value = to_string(temperature);

                voltage_msg.key = "voltage";
                voltage_msg.value = to_string(voltage);

                current_msg.key = "current";
                current_msg.value = to_string(current);

                diagnostic_msg->status[i].values.push_back(pwm_msg);
                diagnostic_msg->status[i].values.push_back(move_msg);
                diagnostic_msg->status[i].values.push_back(temp_msg);
                diagnostic_msg->status[i].values.push_back(voltage_msg);
                diagnostic_msg->status[i].values.push_back(current_msg);
            }

            // update odometry + publish TF?
        

            // publish joint state message, diagnostics message, odometry
            joint_state_publisher_->publish(*joint_state_msg);
            diagnostics_publisher_->publish(*diagnostic_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Configuration parameters mismatch: Joint command messages and Joint ID parameters should be of the same size.");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<PanTiltCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
