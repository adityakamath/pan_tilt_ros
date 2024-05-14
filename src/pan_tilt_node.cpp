#include <cmath>
#include <SCServo_Linux/SCServo.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

using namespace std;

class PanTiltNode : public rclcpp::Node
{
public:
    PanTiltNode() : Node("omni_control_node")
    {
        joint_names_  = this->declare_parameter<vector<string>>("joint_names", {"joint_pan", "joint_tilt"}); 
        speed_        = this->declare_parameter<int>("speed", 4500); // steps per second
        acceleration_ = this->declare_parameter<int>("acceleration", 255); // 0 - 255
        stop_button_  = this->declare_parameter<int>("stop_button", 4); // defaults to SixAxis/SteamDeck:L1

        string           joy_topic  = this->declare_parameter<string>("joy_topic", "/joy");
        vector<long int> joint_ids  = this->declare_parameter<vector<long int>>("joint_ids", {12, 13});
        vector<long int> joy_axes   = this->declare_parameter<vector<long int>>("joy_axes", {0, 1});
        vector<long int> mid_step   = this->declare_parameter<vector<long int>>("mid_pos", {2048, 2048});
        vector<long int> min_step   = this->declare_parameter<vector<long int>>("min_pos", {1280, 1600});
        vector<long int> max_step   = this->declare_parameter<vector<long int>>("max_pos", {2816, 2816});
        int drive_mode              = this->declare_parameter<int>("drive_mode", 0); // 0 = servo, 1 = closed loop wheel, 2 = open loop wheel
        string usb_port             = this->declare_parameter<string>("usb_port", "/dev/ttyACM0");
        int baud_rate               = this->declare_parameter<int>("baud_rate", 1000000); //115200 for sms, 1000000 for sts

        if (joint_names_.size() != 2 || joint_ids.size() != 2 || min_step.size() != 2 || max_step.size() != 2 || mid_step.size() != 2)
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

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            joint_id_map_[joint_names_[i]] = joint_ids[i];
            joint_step_map_[joint_names_[i]] = {mid_step[i], min_step[i], max_step[i]};
            joint_axis_map_[joint_names_[i]] = joy_axes[i];

            uint8_t u8_id = static_cast<uint8_t>(joint_id_map_[joint_names_[i]]);
            st3215_.Mode(u8_id, drive_mode);
            switch (drive_mode)
            {
                case 0:
                    st3215_.RegWritePosEx(u8_id, joint_step_map_[joint_names_[i]][0], speed_, acceleration_);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid mode. Only mode 0 (closed-loop servo mode) is supported.");
                    return;
            }
            stop_ = false;
            st3215_.RegWriteAction();
        }

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic, 10, bind(&PanTiltNode::JoyCallback, this, placeholders::_1));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&PanTiltNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Initialized");
    }

    ~PanTiltNode()
    {
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            stop_ = true;
            st3215_.EnableTorque(static_cast<uint8_t>(joint_id_map_[joint_names_[i]]), 0);
        }
        st3215_.end();
        RCLCPP_INFO(this->get_logger(), "Motors Disabled");
    }
private:
    SMS_STS st3215_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr joy_msg_;

    vector<string> joint_names_;
    int speed_;
    int acceleration_;
    int stop_button_;
    bool stop_;

    unordered_map<string, long int> joint_id_map_;
    unordered_map<string, long int> joint_axis_map_;
    unordered_map<string, vector<long int>> joint_step_map_;

    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if ( msg->buttons[stop_button_] && !joy_msg_->buttons[stop_button_] )
        {
            stop_ = !(stop_);
        }

        joy_msg_ = msg;
    }

    void TimerCallback()
    {
        if (!joy_msg_) {
            RCLCPP_WARN(this->get_logger(), "No joystick message received yet.");
            return;
        }
        
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        auto diagnostic_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
        
        joint_state_msg->header.stamp = this->now();
        
        for(size_t i = 0; i < joint_names_.size(); ++i)
        {
            string j = joint_names_[i];
            double axis_val = -1*joy_msg_->axes[joint_axis_map_[j]];
            uint8_t u8_id = static_cast<uint8_t>(joint_id_map_[j]);
    
            double joint_pos;
            if (axis_val < 0) {
                joint_pos = joint_step_map_[j][1] + (axis_val + 1.0) * (joint_step_map_[j][0] - joint_step_map_[j][1]);
            } else {
                joint_pos = joint_step_map_[j][0] + axis_val * (joint_step_map_[j][2] - joint_step_map_[j][0]);
            }

            // write to servos
            if (!stop_)
            {
                st3215_.RegWritePosEx(u8_id, 
                                      static_cast<int>(round(joint_pos)), 
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
            joint_state_msg->name.push_back(joint_names_[i]);
            joint_state_msg->position.push_back(position);
            joint_state_msg->velocity.push_back(speed);
            joint_state_msg->effort.push_back(current);

            // populate diagnostics message
            diagnostic_msg->status.emplace_back();
            diagnostic_msg->status[i].name = joint_names_[i];
            diagnostic_msg->status[i].hardware_id = to_string(joint_id_map_[joint_names_[i]]);
            diagnostic_msg->status[i].message = joint_names_[i];

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

        // update odometry
        

        // publish joint state message, diagnostics message, odometry
        joint_state_publisher_->publish(*joint_state_msg);
        diagnostics_publisher_->publish(*diagnostic_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<PanTiltNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
