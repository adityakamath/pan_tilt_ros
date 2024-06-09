#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std;

class PanTiltCmdNode : public rclcpp::Node
{
public:
    PanTiltCmdNode() : Node("pan_tilt_cmd_node")
    {
        joint_names_  = this->declare_parameter<vector<string>>("joint_names", {"joint_pan", "joint_tilt"}); 

        vector<long int> mid_step  = this->declare_parameter<vector<long int>>("mid_pos", {2048, 2048});
        vector<long int> min_step  = this->declare_parameter<vector<long int>>("min_pos", {1600, 1600});
        vector<long int> max_step  = this->declare_parameter<vector<long int>>("max_pos", {2816, 2816});
        vector<bool> joint_inv     = this->declare_parameter<vector<bool>>("joint_inv", {false, false});
        
        vector<long int> joy_axes0 = this->declare_parameter<vector<long int>>("joy_axes0", {2, 5});
        vector<long int> joy_axes1 = this->declare_parameter<vector<long int>>("joy_axes1", {4});
        vector<vector<long int>> joy_axes = {joy_axes0, joy_axes1};

        if (joint_names_.size() != 2 || min_step.size() != 2 || max_step.size() != 2 || mid_step.size() != 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameters. 2 joint names, IDs and angles are required. Currently only two motor (pan-tilt) configurations are supported");
            return;
        }

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            joint_step_map_[joint_names_[i]] = {mid_step[i], min_step[i], max_step[i]};
            joint_axis_map_[joint_names_[i]] = joy_axes[i];
            joint_inv_map_[joint_names_[i]] = joint_inv[i];
        }

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&PanTiltCmdNode::JoyCallback, this, placeholders::_1));
        joint_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&PanTiltCmdNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Initialized");
    }

    ~PanTiltCmdNode()
    {
    }

private:
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr joy_msg_;

    vector<string> joint_names_;

    unordered_map<string, vector<long int>> joint_axis_map_;
    unordered_map<string, vector<long int>> joint_step_map_;
    unordered_map<string, bool> joint_inv_map_;

    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        joy_msg_ = msg;
    }

    void TimerCallback()
    {
        if (!joy_msg_) {
            RCLCPP_WARN(this->get_logger(), "No joystick message received yet.");
            return;
        }
        
        auto joint_cmd_msg = std::make_shared<sensor_msgs::msg::JointState>();
        
        joint_cmd_msg->header.stamp = this->now();
        
        for(size_t i = 0; i < joint_names_.size(); ++i)
        {
            string j = joint_names_[i];
            double axis_val = 0.0;
            switch(joint_axis_map_[j].size())
            {
                case 1:
                    axis_val = -1*joy_msg_->axes[joint_axis_map_[j][0]];
                    break;
                case 2:
                    axis_val = -1*(joy_msg_->axes[joint_axis_map_[j][1]] - joy_msg_->axes[joint_axis_map_[j][0]])/2;
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid axis vector size. The axis vector supports only 1 or 2 values");
                    break;
            }

            if(joint_inv_map_[j])
            {
                axis_val = -1*axis_val;
            }
    
            double joint_pos;
            if (axis_val < 0) {
                joint_pos = joint_step_map_[j][1] + (axis_val + 1.0) * (joint_step_map_[j][0] - joint_step_map_[j][1]);
            } else {
                joint_pos = joint_step_map_[j][0] + axis_val * (joint_step_map_[j][2] - joint_step_map_[j][0]);
            }

            // populate joint state message
            joint_cmd_msg->name.push_back(joint_names_[i]);
            joint_cmd_msg->position.push_back(joint_pos*2*M_PI/4096.0);
        }

        // publish joint command message
        joint_cmd_publisher_->publish(*joint_cmd_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<PanTiltCmdNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
