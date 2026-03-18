---
title: Loaned message and Zero copy
tags:
    - zero copy
    - borrow
    - loan message
    - can_loan_messages
    - borrow_loaned_message
---

## Loaned message
Who owns the buffer and who allocates it?

- Must follow POD (Plain Old Data)
- Avoid allocation
- Reuse memory
- Control by rclcpp, rcl, rmw

```cpp
auto msg = pub->borrow_loaned_message();
```

!! tip
    Loaned messages are publisher-side only
    

!!! tip "POD"
    A message that can be represented as a fixed, contiguous block of memory

    ```
    float64 x
    float64 y
    float64 z
    ```

    ```
    int32[10] values
    uint8 flag
    ```


### Demo: 

<details>
<summary>Publihser</summary>

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "my_msgs/msg/num.hpp"

using namespace std::chrono_literals;

class PubNode : public rclcpp::Node
{
public:
  PubNode()
  : Node("string_loan_check")
  {
    publisher_ = this->create_publisher<my_msgs::msg::Num>("numbers", 10);

    const bool loan_supported = publisher_->can_loan_messages();

    RCLCPP_INFO(this->get_logger(), "Topic: %s", publisher_->get_topic_name());
    RCLCPP_INFO(
      this->get_logger(),
      "can_loan_messages() = %s",
      loan_supported ? "true" : "false");

    if (!loan_supported) {
      RCLCPP_WARN(
        this->get_logger(),
        "Loaned messages are not supported for this publisher/RMW/type. "
        "Will publish with normal copy-based message.");
    }

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&PubNode::on_timer, this));
  }

private:
  void on_timer()
  {
    my_msgs::msg::Num msg;
    msg.data = 42;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.data);
    publisher_->publish(msg);

    auto msg = publisher_->borrow_loaned_message();
    msg.get().data = 42;
    publisher_->publish(std::move(msg));
  }

  rclcpp::Publisher<my_msgs::msg::Num>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubNode>());
  rclcpp::shutdown();
  return 0;
}
```
</details>
---

## Zero-copy
Use same memory for pub and sub

- Control by DDS/Transport (FastDDS SHM)
- Avoid copying between publisher and subscriber

---

| Loaned | SHM transport | Result                                 |
| ------ | ------------- | -------------------------------------- |
| ❌ No   | ❌ No          | ❌ Copy + allocation                    |
| ✅ Yes  | ❌ No          | ⚠️ No allocation, but still copies     |
| ❌ No   | ✅ Yes         | ⚠️ Possible SHM copy inside middleware |
| ✅ Yes  | ✅ Yes         | ✅ TRUE zero-copy                       |


---

## Reference
- [Configure Zero Copy Loaned Messages](https://docs.ros.org/en/jazzy/How-To-Guides/Configure-ZeroCopy-loaned-messages.html)
- [fast dds zero copy shm](https://github.com/ros2/rmw_fastrtps?tab=readme-ov-file#enable-zero-copy-data-sharing)