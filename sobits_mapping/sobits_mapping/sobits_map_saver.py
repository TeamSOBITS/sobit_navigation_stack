import rclpy
from rclpy.node import Node
import os
from subprocess import Popen, PIPE

def save_map_command(node):
    proc = Popen(["zenity", "--file-selection", "--save", "--confirm-overwrite", "--filename=/home/" + str(os.getenv("USER")) + "/colcon_ws/src/sobits_navigation_stack/sobits_mapping/map/map_name.yaml"],
        stdout=PIPE,
        shell=False)
    out, err = proc.communicate()
    if (str(out.decode('utf-8')) == ""):
        print("\033[91m\033[05mNONE FILE PATH\033[0m")
        node.get_logger().info('\033[91m\033[05mNONE FILE PATH\033[0m')
        return False, ""
    else:
        if (len(out.decode('utf-8').split("."))==1):
            path = ".".join(out.decode('utf-8').split("."))
        else:
            path = ".".join(out.decode('utf-8').split(".")[:-1])
        print("MAP FILE :\033[93m\033[05m", path, "\033[0m")
        node.get_logger().info('MAP FILE :\033[93m\033[05m'+path+'\033[0m')
        return True, path


def main(args=None):
    rclpy.init(args=args)
    node = Node("sobits_map_saver")
    while rclpy.ok():
        r, path = save_map_command(node)
        if r:
            Popen(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path])
            Popen(["sed", "-i", "s/free_thresh: 0.25/free_thresh: 0.196/", path + ".yaml"])
    node.execute()
    rclpy.shutdown()

if __name__ == '__main__':
    main()