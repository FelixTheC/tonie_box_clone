#include "mfrc522_reader.hpp"
#include <wiringPi.h>


using std::chrono::milliseconds;

Mfrc522ReaderNode::Mfrc522ReaderNode(): Node("mfrc522_reader")
{
    if (wiringPiSetup() == -1)
        RCLCPP_INFO(get_logger(), "Wiring Pi setup failed.");

    publisher_ = create_publisher<StringMsg>("mfrc_connection", 10);
    timer_ = create_wall_timer(
        milliseconds(1000),
        std::bind(&Mfrc522ReaderNode::publishMFRC522Connection, this)
    );
    
    RCLCPP_INFO(get_logger(), "Mfrc522ReaderNode has been started.");
}

void 
Mfrc522ReaderNode::publishMFRC522Connection()
{
    auto msg = StringMsg();

    auto mfrc_delegate = MFRC522_Delegate();
    auto mfrc = mfrc_delegate.get_mfrc();

    if (mfrc_delegate.readable_card_detected())
    {
        RCLCPP_INFO(get_logger(), "Mfrc522ReaderNode new card detected.");
        
        mfrc_delegate.uid_bytes_to_string(mfrc);
        msg.data = "{\"status\": \"ok\", \"card_id\": \"" +  mfrc_delegate.get_card_uid() + "\"}";
        mfrc_delegate.halt_picc();
    }
    else
    {
        msg.data = "{\"status\": \"failed\"}";
    }

    publisher_->publish(msg);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Mfrc522ReaderNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}