#!/usr/bin/python3
# -*- coding:utf-8 -*-

import glob
import os
import sys
# try:
#     sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
#     sys.path.append("./")
# except IndexError:
#     pass
sys.path.append('/home/xzh/software/carla014/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')
# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
import numpy as np
import random
import rclpy
from rclpy.node import Node
from ros_g29_force_feedback.msg import ForceFeedback

class CarlaFFNode(Node):
    def __init__(self, client):
        super().__init__("carla_ff_node")
        self.publisher = self.create_publisher(ForceFeedback,"/ff_target", 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.client = client
        self.world = client.get_world()
        self.actor = self.get_hero()
        self.previous_angle = 0

    def get_hero(self):
        for actor in self.world.get_actors():
            if actor.attributes.get("role_name") in ["hero", "ego_vehicle"]:
                return actor
            
    def timer_cb(self):
        out_msg = ForceFeedback()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        
        # steering_angle = self.actor.get_control().steer
        
        # 获取当前的steering_angle值
        current_angle = self.actor.get_control().steer
        # 设置平滑系数alpha（范围在0到1之间，值越接近1，平滑效果越明显，但响应速度越慢）
        alpha = 0.2

        # 计算平滑后的steering_angle值
        smoothed_angle = alpha * current_angle + (1 - alpha) * self.previous_angle

        # 更新previous_angle的值为当前平滑后的值，以便下一次计算
        self.previous_angle = smoothed_angle

        # 将平滑后的steering_angle值赋给out_msg的position字段
        out_msg.position = smoothed_angle        
        
        # out_msg.position = steering_angle
        out_msg.torque = 0.8
        self.publisher.publish(out_msg)

def main(args=None):

    rclpy.init(args=args)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(200.0)
    print("running.....")
    carla_ff_node = CarlaFFNode(client)
    rclpy.spin(carla_ff_node)
    carla_ff_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
