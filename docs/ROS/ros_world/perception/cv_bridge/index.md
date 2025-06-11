---
tags:
    - cv_bridge
    - ros
    - opencv
---
## cv bridge

<div class="grid-container">
    <div class="grid-item">
            <a href="cv_bridge_build">
                <img src="images/build.png"  width="150" height="150">
                <p>Build</p>
            </a>
        </div>
        <div class="grid-item">
             <a href="">
                <!-- <img src="images/quaternion.png"  width="150" height="150"> -->
                <p>TBD</p>
            </a>
        </div>
    <div class="grid-item">
          <a href="">
                <!-- <img src="images/quaternion.png"  width="150" height="150"> -->
                <p>TBD</p>
            </a>
    </div>

</div>



```bash title="install"
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport python3-opencv

```


```python title="simple demo"
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change to your actual image topic
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Show image using OpenCV
            cv2.imshow('Camera Image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

```