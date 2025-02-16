#!/bin/python


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo


class CameraInfoClient(Node):
    def __init__(self):
        super().__init__('camera_info_client')
        self.client = self.create_client(SetCameraInfo, '/set_camera_info')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Waiting for /set_camera_info service...")

        self.send_request()

    def send_request(self):
        request = SetCameraInfo.Request()
        
        # Fill the CameraInfo message
        request.camera_info = CameraInfo()
        request.camera_info.width = 1280
        request.camera_info.height = 720
        request.camera_info.k = [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0]  # Intrinsic matrix
        request.camera_info.d = [0.1, -0.05, 0.001, 0.002, 0.0]  # Distortion coefficients
        request.camera_info.distortion_model = "plumb_bob"

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Camera info updated successfully!")
            else:
                self.get_logger().error("Failed to update camera info!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main():
    rclpy.init()
    node = CameraInfoClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
