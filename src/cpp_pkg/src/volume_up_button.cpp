#include "button_node.hpp"

ButtonNode::ButtonNode() : Node("volume_up_node")
{
    rclcpp::Rate rate(10);
    auto duration = rclcpp::Duration(1.0).to_chrono<milliseconds>();

    _publisher = create_publisher<BoolMsg>("volume_up", 1);
    _wallTimer = create_wall_timer(
        rate.period(),
        std::bind(&ButtonNode::publishButtonStatus, this));
    ButtonPin = 22;

    setupButton();
}

void 
ButtonNode::setupButton()
{
    pinMode(ButtonPin, INPUT);
    pullUpDnControl(ButtonPin, PUD_DOWN);
}

void 
ButtonNode::publishButtonStatus()
{
    auto msg = BoolMsg();
    if (digitalRead(ButtonPin) == 0)
    {
        msg.data = true;
        RCLCPP_INFO(get_logger(), "Volume Up Button pressed");
    }
    else
    {
        msg.data = false;
    }
    
    _publisher->publish(msg);
}


int main(int argc, char **argv)
{
    if(wiringPiSetup() == -1)
    { //when initialize wiring failed,print message to screen
		std::cout << "setup wiringPi failed !\n";
		return 1; 
	}

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ButtonNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
