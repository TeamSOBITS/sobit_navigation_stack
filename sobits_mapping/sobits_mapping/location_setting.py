import rclpy
from rclpy.node import Node
import os
import time
from ament_index_python.packages import get_package_share_directory
import yaml
from subprocess import Popen, PIPE
import tkinter as tk
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import SetInitialPose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
import threading





class LocationSetting(Node):
    def __init__(self):
        super().__init__('create_location_file')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare parameters
        self.declare_parameter('use_robot', True)
        self.declare_parameter('robot_name', '')

        # Get parameters
        self.use_robot = self.get_parameter('use_robot').get_parameter_value().bool_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.pub_location_path = self.create_publisher(String, "/location_file_path", 1)

        # if (not self.use_robot):
        #     self.sub_2d_nav_goal = self.create_subscription(PoseStamped, "/goal_pose", self.callback_nav_goal, 1)

        self.client_add_location = self.create_client(SetInitialPose, "/add_location")
        self.client_delete_location = self.create_client(SetInitialPose, "/delete_location")
        while not ((self.client_add_location.wait_for_service(timeout_sec=1.0)) and (self.client_delete_location.wait_for_service(timeout_sec=1.0))):
            self.get_logger().info('service not available, waiting again...')
        self.set_pose_req = SetInitialPose.Request()

        self.tk = tk.Tk()
        self.iconfile = tk.PhotoImage(file=os.path.join(get_package_share_directory('sobits_mapping'), 'img', 'mapping.png'))
        self.width = self.tk.winfo_screenwidth()
        self.height = self.tk.winfo_screenheight()
        self.tk.call('wm', 'iconphoto', self.tk._w, self.iconfile)

        self.location_path = ""
        self.location_path_flag = False

        self.sub_ctrl_now = False

        self.location_poses = {}


    def callback_nav_goal(self, msg):
        # self.get_logger().info("\033[31m====================================\033[0m")
        if ((not self.location_path_flag) or (self.sub_ctrl_now)):
            return
        self.sub_ctrl_now = True
        # self.sub_tk = tk.Tk()
        def show_sub_gui():
            self.sub_tk = tk.Toplevel(self.tk)
            iconfile = tk.PhotoImage(file=os.path.join(get_package_share_directory('sobits_mapping'), 'img', 'mapping.png'))
            width = self.sub_tk.winfo_screenwidth()
            height = self.sub_tk.winfo_screenheight()
            # self.sub_tk.call('wm', 'iconphoto', self.sub_tk._w, iconfile)
            self.sub_tk.iconphoto(False, iconfile)
            geometry_x = 400
            geometry_y = 300

            # ウィンドウ位置を中央に配置
            self.sub_tk.geometry(f"{geometry_x}x{geometry_y}+{(self.width - geometry_x) // 2}+{(self.height - geometry_y) // 2}")

            tk.Label(self.sub_tk, text="New Location Name : ", font=("", 15)).place(x=80, y=150)
            entry = tk.Entry(self.sub_tk, width=24, font=("", 12))
            entry.place(x=80, y=200)
            tk.Button(self.sub_tk, width=6, text="Cancel", command=lambda : self.button_clicked_callback_sub("cancel")).place(x=80, y=220)
            tk.Button(self.sub_tk, width=6, text="Set",    command=lambda entry=entry, pose_msg=msg: self.button_clicked_callback_sub("set", entry, pose_msg)).place(x=200, y=220)

            # text = entry.get()
            self.sub_tk.title("[ENTER] New Location Name??")
        # self.sub_tk.mainloop()
        self.tk.after(0, show_sub_gui)


    def reset_locations_info(self):
        if ((not self.location_path_flag) or (not os.path.isfile(self.location_path + ".yaml"))):
            self.location_poses = {}
            return
        with open(self.location_path + ".yaml", "r") as f:
            yaml_value = yaml.safe_load(f)["location_pose"]

        self.location_poses = {}

        if (yaml_value is not None):
            for location_name in yaml_value.keys():
                pose = Pose()
                pose.position.x = yaml_value[str(location_name)]["translation"]["x"]
                pose.position.y = yaml_value[str(location_name)]["translation"]["y"]
                pose.position.z = yaml_value[str(location_name)]["translation"]["z"]
                pose.orientation.x = yaml_value[str(location_name)]["rotation"]["x"]
                pose.orientation.y = yaml_value[str(location_name)]["rotation"]["y"]
                pose.orientation.z = yaml_value[str(location_name)]["rotation"]["z"]
                pose.orientation.w = yaml_value[str(location_name)]["rotation"]["w"]
                if (str(location_name) not in self.location_poses):
                    self.location_poses[str(location_name)] = pose


    def create_gui(self):
        # GUIウィンドウの大きさを定義する
        geometry_x = 510
        if (len(self.location_poses) <= 1):
            geometry_y = 30 * 2
        else:
            if (self.use_robot):
                geometry_y = 30 * (len(self.location_poses) + 1)
            else:
                geometry_y = 30 * (len(self.location_poses))


        # ウィンドウ位置を中央に配置
        # self.tk.geometry(f"{geometry_x}x{geometry_y}+{(self.width - geometry_x) // 2}+{(self.height - geometry_y) // 2}")
        # ウィンドウ位置を右上に配置
        # self.tk.geometry(f"{geometry_x}x{geometry_y}+0+{(self.height - geometry_y) // 2}")
        self.tk.geometry(f"{geometry_x}x{geometry_y}+0+0")
        # self.tk.geometry(f"{geometry_x}x{geometry_y}+{geometry_x // 2}+{geometry_y // 2}")

        i = 0
        for k in self.location_poses.keys():
            # tk.Label(text=container_info, font=("", 15)).place(x=460, y=i * 30)
            entry = tk.Entry(self.tk, width=24, font=("", 12))
            entry.insert(0, k)
            entry.place(x=250, y=i * 30 + 3)

            tk.Button(self.tk, width=7, text="Delete", command=lambda k=k: self.button_clicked_callback("delete", k)).place(x=76, y=i * 30)
            tk.Button(self.tk, width=7, text="Rename", command=lambda k=k, entry=entry: self.button_clicked_callback("rename", k, entry)).place(x=162, y=i * 30)

            i += 1

        if (self.use_robot):
            # tk.Button(self.tk, width=49, text="ADD LOCATION", command=lambda: self.button_clicked_callback("add", "")).place(x=76, y=i * 30)
            tk.Button(self.tk, width=49, text="ADD LOCATION", command=lambda: self.get_robot_position()).place(x=76, y=i * 30)


        # GUI再起動用ボタン
        tk.Button(self.tk, width=4, text="refresh", command=self.refresh_gui).place(x=0, y=0)
        # GUI停止用ボタン
        tk.Button(self.tk, width=4, text="close", command=self.quit_gui).place(x=0, y=30)

        self.tk.title("[Location] Location Setting GUI")
        self.tk.mainloop()


    def button_clicked_callback(self, mode, name, replace_word=None):
        if (self.sub_ctrl_now):
            return
        if (mode == "delete"):
            self.get_logger().info("[DELETE] : " + str(name))

            self.set_pose_req.pose.header.frame_id = str(name)

            future = self.client_delete_location.call_async(self.set_pose_req)
            # rclpy.spin_until_future_complete(self, future)
            time.sleep(0.5) ## TODO

        elif ((mode == "rename") and (str(replace_word.get()) not in self.location_poses)):
            ## DELETE ##
            self.get_logger().info("[RENAME] : " + str(name) + " to " + str(replace_word.get()))
            self.set_pose_req.pose.header.frame_id = str(name)
            future = self.client_delete_location.call_async(self.set_pose_req)

            ## ADD ##
            self.set_pose_req.pose.header.frame_id = str(replace_word.get())
            self.set_pose_req.pose.pose.pose.position.x = self.location_poses[str(name)].position.x
            self.set_pose_req.pose.pose.pose.position.y = self.location_poses[str(name)].position.y
            self.set_pose_req.pose.pose.pose.position.z = self.location_poses[str(name)].position.z
            self.set_pose_req.pose.pose.pose.orientation.x = self.location_poses[str(name)].orientation.x
            self.set_pose_req.pose.pose.pose.orientation.y = self.location_poses[str(name)].orientation.y
            self.set_pose_req.pose.pose.pose.orientation.z = self.location_poses[str(name)].orientation.z
            self.set_pose_req.pose.pose.pose.orientation.w = self.location_poses[str(name)].orientation.w

            future = self.client_add_location.call_async(self.set_pose_req)
            time.sleep(0.5) ## TODO
        # elif (mode == "add"):
        #     self.get_logger().info("[ADD]")
        self.refresh_gui()


    def button_clicked_callback_sub(self, mode, new_name=None, pose=None):
        # if (mode == "cancel"):
        if ((mode == "set") and (str(new_name.get()) != "")):
            self.get_logger().info("[ADD] : " + str(new_name.get()))
            self.set_pose_req.pose.header.frame_id = str(new_name.get())
            self.set_pose_req.pose.pose.pose.position.x = pose.pose.position.x
            self.set_pose_req.pose.pose.pose.position.y = pose.pose.position.y
            self.set_pose_req.pose.pose.pose.position.z = pose.pose.position.z
            self.set_pose_req.pose.pose.pose.orientation.x = pose.pose.orientation.x
            self.set_pose_req.pose.pose.pose.orientation.y = pose.pose.orientation.y
            self.set_pose_req.pose.pose.pose.orientation.z = pose.pose.orientation.z
            self.set_pose_req.pose.pose.pose.orientation.w = pose.pose.orientation.w

            future = self.client_add_location.call_async(self.set_pose_req)
            # rclpy.spin_until_future_complete(self, future)
            # future.result()
            time.sleep(0.5)
        # self.sub_tk.quit()
        self.sub_tk.destroy()
        self.sub_ctrl_now = False
        self.refresh_gui()


    def refresh_gui(self):
        self.reset_locations_info()

        if (self.location_path_flag):
            data = String()
            data.data = self.location_path + ".yaml"
            self.pub_location_path.publish(data)

        self.tk.quit()
        self.tk.destroy()

        self.tk = tk.Tk()
        self.iconfile = tk.PhotoImage(file=os.path.join(get_package_share_directory('sobits_mapping'), 'img', 'mapping.png'))
        self.width = self.tk.winfo_screenwidth()
        self.height = self.tk.winfo_screenheight()
        self.tk.call('wm', 'iconphoto', self.tk._w, self.iconfile)

        self.create_gui()


    def quit_gui(self):
        self.tk.quit()
        self.tk.destroy()

    
    def get_robot_position(self):
        # Wait for the transform to be available
        detection = False
        goal_pose = PoseStamped()
        if (self.robot_name != ""):
            # while not self.tf_buffer.can_transform('map', self.robot_name + '/base_footprint', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0)):
            #     self.get_logger().info("Waiting for transform from %s to map" % (self.robot_name + '/base_footprint'))
                # rclpy.spin_once(self)


            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    self.robot_name + '/base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.time.Duration(seconds=3.0),
                )

                goal_pose.header.frame_id    = ""
                goal_pose.header.stamp       = self.get_clock().now().to_msg()
                goal_pose.pose.position.x    = transform.transform.translation.x
                goal_pose.pose.position.y    = transform.transform.translation.y
                goal_pose.pose.position.z    = transform.transform.translation.z
                goal_pose.pose.orientation.x = transform.transform.rotation.x
                goal_pose.pose.orientation.y = transform.transform.rotation.y
                goal_pose.pose.orientation.z = transform.transform.rotation.z
                goal_pose.pose.orientation.w = transform.transform.rotation.w

                detection = True

            except TransformException as e:
                self.get_logger().error("Transform error: %s" % e)
        if (not detection):
            # while not self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0)):
            #     self.get_logger().info("Waiting for transform from %s to map" % 'base_footprint')
                # rclpy.spin_once(self)

            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.time.Duration(seconds=3.0),
                )

                goal_pose.header.frame_id    = ""
                goal_pose.header.stamp       = self.get_clock().now().to_msg()
                goal_pose.pose.position.x    = transform.transform.translation.x
                goal_pose.pose.position.y    = transform.transform.translation.y
                goal_pose.pose.position.z    = transform.transform.translation.z
                goal_pose.pose.orientation.x = transform.transform.rotation.x
                goal_pose.pose.orientation.y = transform.transform.rotation.y
                goal_pose.pose.orientation.z = transform.transform.rotation.z
                goal_pose.pose.orientation.w = transform.transform.rotation.w

                detection = True

            except TransformException as e:
                self.get_logger().error("Transform error: %s" % e)
        
        if (not detection):
            self.get_logger().error("\033[31mNo Robot Connection...\033[0m")
            return
        self.callback_nav_goal(goal_pose)


    def select_location_file(self):
        proc = Popen(["zenity", "--file-selection", "--save", "--confirm-overwrite", "--filename=/home/" + str(os.getenv("USER")) + "/colcon_ws/src/sobits_navigation_stack/sobits_mapping/location/location_example.yaml"],
            stdout=PIPE,
            shell=False)
        out, err = proc.communicate()
        if (str(out.decode('utf-8')) == ""):
            self.location_path = ""
            # print("\033[91m\033[05mNONE FILE PATH\033[0m")
            self.get_logger().info('\033[91m\033[05mNONE FILE PATH\033[0m')
            self.location_path_flag = False
            # return False, ""
        else:
            if (len(out.decode('utf-8').split(".")) == 1):
                self.location_path = ".".join(out.decode('utf-8').split("."))
            else:
                self.location_path = ".".join(out.decode('utf-8').split(".")[:-1])
            # print("LOCATION FILE :\033[93m\033[05m", self.location_path, "\033[0m")
            self.get_logger().info('LOCATION FILE :\033[93m\033[05m'+self.location_path+'\033[0m')
            self.location_path_flag = True

            data = String()
            data.data = self.location_path + ".yaml"
            self.pub_location_path.publish(data)
            # return True, self.location_path


class CallbackGroup:
    def __init__(self, main_node: LocationSetting):
        self.main_node = main_node

        use_robot = self.main_node.get_parameter('use_robot').get_parameter_value().bool_value
        if not use_robot:
            self.main_node.sub_2d_nav_goal = self.main_node.create_subscription(
                PoseStamped, "/goal_pose", self.main_node.callback_nav_goal, 1)


def main():
    rclpy.init()
    node = LocationSetting()

    sub_node = CallbackGroup(node)

    while rclpy.ok():
        node.select_location_file()
        if node.location_path_flag:
            break

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    node.reset_locations_info()
    node.create_gui()

    node.destroy_node()
    # sub_node.destroy_node()
    rclpy.shutdown()

    # node = Node("sobits_map_saver")
    # while rclpy.ok():
    #     r, path = select_location_file(node)
    #     if r:
    #         Popen(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path])
    #         Popen(["sed", "-i", "s/free_thresh: 0.25/free_thresh: 0.196/", path + ".yaml"])
    # node.execute()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
思うように動作しなくて，恐らくこれが原因で動きません
これはスレッドが作られるタイミングで発生します
```
[WARN] [1747906461.446574846] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
```
"""