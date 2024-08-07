import sys

sys.path.append("/home/sobits/colcon_ws/src/kachaka-api/python")

import kachaka_api
import yaml
from scipy.spatial.transform import Rotation as R


# 地点が記録されたのファイルのパス
location_path = "/home/sobits/colcon_ws/src/sobit_navigation_stack/location/location.yaml"


def move(location):
    client = kachaka_api.KachakaApiClient("192.168.11.19:26400")

    client.speak(location + "に移動します！")
    try:
        x,y,rad = get_location(location)
        client.move_to_pose(x, y, rad)
    except:
        print(location + "は地点登録されていません")


    # 充電ドックに戻らないモードに変更する
    # client.set_auto_homing_enabled(False)

    # 充電ドックに戻るモードに変更する
    # client.set_auto_homing_enabled(True)

    
# 地点名→x,y,rad
def get_location(location):
    global location_path

    with open(location_path, 'r') as yml:
        config = yaml.safe_load(yml)

    num = -1
    for i in range(len(config)):
        if config["location_pose"][i]["location_name"] == location:
            num = i
            break
    
    # 地点登録されていなかった場合
    if num == -1:
        return
    
    # クオータニオン→オイラー角
    quat = R.from_quat([config["location_pose"][num]["rotation_x"], config["location_pose"][num]["rotation_y"], config["location_pose"][num]["rotation_z"], config["location_pose"][num]["rotation_w"]])
    euler =  quat.as_euler('zyx', degrees=True)
    
    x = config["location_pose"][num]["translation_x"]
    y = config["location_pose"][num]["translation_y"]
    rad = euler[0]

    return x,y,rad


if __name__ == "__main__":
   move("kitchen")