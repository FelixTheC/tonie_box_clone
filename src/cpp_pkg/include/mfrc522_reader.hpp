#ifndef MFRC522_READER_HPP
#define MFRC522_READER_HPP

#include <pigpio.h>
#include <mfrc522_delegate.h>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using rclcpp::TimerBase;
using rclcpp::Publisher;
using StringMsg=example_interfaces::msg::String;

class Mfrc522ReaderNode : public rclcpp::Node
{

private:
    TimerBase::SharedPtr timer_;
    Publisher<StringMsg>::SharedPtr publisher_;

public:
    Mfrc522ReaderNode();

    void publishMFRC522Connection();

};

#endif