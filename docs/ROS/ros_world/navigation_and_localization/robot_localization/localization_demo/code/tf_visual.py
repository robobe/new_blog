#!/usr/bin/env python3
# https://github.com/Kapernikov/ros_robot_localization_tutorial/blob/master/ros-ws/src/robot_localization_demo/src/transforms/transformation_visualization_node.cpp

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from tf2_ros import TransformListener, Buffer
import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose

SRV_SPAWN = "/spawn"
ROBOT_NAME = "filter_turtle"
SRV_TELEPORT = f"/{ROBOT_NAME}/teleport_absolute"

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.__last_call = 0
        self.__update_counter = 0
        self.__spawn_client = self.create_client(Spawn, SRV_SPAWN)
        while not self.__spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service not available. Retrying... ")

        self.__spawn()
        self.__teleport_client = self.create_client(TeleportAbsolute, SRV_TELEPORT)
        while not self.__teleport_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"Service teleport not available. Retrying... ")

        # Create a buffer and a listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically lookup transform
        self.timer = self.create_timer(1.0, self.__timer_callback)

    def __spawn(self):
        request = Spawn.Request()
        request.name = ROBOT_NAME
        request.x = 5.55
        request.y = 5.55
        future = self.__spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Turtle spawned successfully: {future.result()}")
        else:
            self.get_logger().error(f"Failed to spawn turtle: {future.exception()}")

        SRV_SET_PEN = f"/{ROBOT_NAME}/set_pen"
        set_pen_client = self.create_client(SetPen, SRV_SET_PEN)
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service set_pen not available. Retrying... ")

        request = SetPen.Request()
        request.r = 0
        request.g = 255 
        request.b = 0
        request.width = 3
        request.off = 0
        future = set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Turtle 'set pen' successfully: {future.result()}")
        else:
            self.get_logger().error(f"Failed to 'set pen' turtle: {future.exception()}")


    def __timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link',
                time=Time(),  # "now"
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().info(
                f"Transform: Translation = ({trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f}, {trans.transform.translation.z:.2f})"
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform: {e}')


        pose = Pose()
        new_pose = do_transform_pose(pose, trans)
        request = TeleportAbsolute.Request()
        request.x = new_pose.position.x
        request.y = new_pose.position.y
        request.theta = 0.0
        future = self.__teleport_client.call_async(request)
        future.add_done_callback(self.__teleport_response_handler)

    def __teleport_response_handler(self, future):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()