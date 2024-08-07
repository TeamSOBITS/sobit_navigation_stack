#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
from scipy.spatial.transform import Rotation as R
from kachaka_api import KachakaApiClient  # Adjust the import path as necessary

# 地点が記録されたのファイルのパス
location_path = "/home/sobits/colcon_ws/src/sobit_navigation_stack/location/location.yaml"


class RobotMover(Node):

    def __init__(self):
        super().__init__('robot_mover')
        self.client = KachakaApiClient("192.168.11.19:26400")

    def move(self, location):
        self.client.speak(location + "に移動します！")
        try:
            x, y, rad = self.get_location(location)
            self.client.move_to_pose(x, y, rad)
        except Exception as e:
            self.get_logger().error(f"{location}は地点登録されていません: {e}")

    def get_location(self, location):
        global location_path

        with open(location_path, 'r') as yml:
            config = yaml.safe_load(yml)

        num = -1
        for i in range(len(config["location_pose"])):
            if config["location_pose"][i]["location_name"] == location:
                num = i
                break

        # 地点登録されていなかった場合
        if num == -1:
            raise ValueError("Location not registered")

        # クオータニオン→オイラー角
        quat = R.from_quat([
            config["location_pose"][num]["rotation_x"],
            config["location_pose"][num]["rotation_y"],
            config["location_pose"][num]["rotation_z"],
            config["location_pose"][num]["rotation_w"]
        ])
        euler = quat.as_euler('zyx', degrees=True)

        x = config["location_pose"][num]["translation_x"]
        y = config["location_pose"][num]["translation_y"]
        rad = euler[0]

        return x, y, rad


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    # 登録された地点名に変更する
    node.move("b")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
