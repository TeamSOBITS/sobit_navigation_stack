#!/usr/bin/env python3

import math
import os
import shutil
import threading
import tkinter as tk
from pathlib import Path
from subprocess import PIPE, Popen

import rclpy
import yaml
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class _RoomYamlDumper(yaml.SafeDumper):
    pass


class _QuotedStr(str):
    pass


def _represent_list_with_inline_points(dumper, data):
    if len(data) == 2 and all(isinstance(value, (int, float)) for value in data):
        return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data)


_RoomYamlDumper.add_representer(list, _represent_list_with_inline_points)
_RoomYamlDumper.add_representer(
    _QuotedStr,
    lambda dumper, data: dumper.represent_scalar("tag:yaml.org,2002:str", str(data), style='"'),
)


def _default_room_info_path() -> Path:
    return Path.cwd() / "room_information.yaml"


def _zenity_default_room_info_path() -> Path:
    username = os.getenv("USERNAME") or os.getenv("USER")
    if username:
        return Path("/home") / username / "colcon_ws" / "src" / "sobits_navigation_stack" / "sobits_slam" / "location" / "room_information_example.yaml"
    return Path.home() / "colcon_ws" / "src" / "sobits_navigation_stack" / "sobits_slam" / "location" / "room_information_example.yaml"


def _normalize_room_name(raw_name: str) -> str:
    return raw_name.strip().replace(" ", "_")


def _sort_points_clockwise(points):
    if len(points) <= 2:
        return list(points)

    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    return sorted(points, key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x))


def _room_color(index: int):
    palette = [
        (0.91, 0.30, 0.24),
        (0.18, 0.80, 0.44),
        (0.20, 0.60, 0.86),
        (0.95, 0.77, 0.06),
        (0.61, 0.35, 0.71),
        (0.10, 0.74, 0.61),
        (0.90, 0.49, 0.13),
        (0.80, 0.36, 0.36),
    ]
    return palette[index % len(palette)]


class RoomPolygonSetting(Node):
    def __init__(self):
        super().__init__("room_polygon_setting")

        self.declare_parameter("config_path", "")
        self.config_path = ""

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            1,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/room_polygon_markers", marker_qos)

        self.lock = threading.Lock()
        self.room_polygons = {}
        self.selected_room = None

        self.tk = tk.Tk()
        self.tk.title("[Room Polygon] Map Information Setting GUI")
        self.width = self.tk.winfo_screenwidth()
        self.height = self.tk.winfo_screenheight()

        self.status_var = tk.StringVar(value="Select a room file to begin.")
        self.room_listbox = None
        self.point_listbox = None

    def goal_pose_callback(self, msg: PoseStamped):
        point = (round(float(msg.pose.position.x), 3), round(float(msg.pose.position.y), 3))
        self.tk.after(0, lambda: self.append_point_to_selected_room(point, source="/goal_pose"))

    def select_config_file(self):
        default_path = str(_zenity_default_room_info_path())
        proc = Popen(
            [
                "zenity",
                "--file-selection",
                "--save",
                "--confirm-overwrite",
                f"--filename={default_path}",
            ],
            stdout=PIPE,
            shell=False,
        )
        out, _ = proc.communicate()
        selected = out.decode("utf-8").strip()

        if selected:
            self.config_path = selected
        else:
            self.config_path = default_path
            self.get_logger().info(f"Zenity returned no path. Using default room file: {self.config_path}")

    def load_config(self):
        if not self.config_path:
            self.config_path = str(_default_room_info_path())

        config_path = Path(self.config_path)
        if not config_path.exists():
            self.room_polygons = {}
            self.status_var.set(f"New room file selected: {config_path}")
            return

        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}

        loaded_polygons = config.get("room_polygons", {}) or {}
        with self.lock:
            self.room_polygons = {
                room_name: [
                    (round(float(point[0]), 3), round(float(point[1]), 3))
                    for point in polygon
                    if isinstance(point, (list, tuple)) and len(point) >= 2
                ]
                + [
                    (round(float(point["x"]), 3), round(float(point["y"]), 3))
                    for point in polygon
                    if isinstance(point, dict) and "x" in point and "y" in point
                ]
                for room_name, polygon in loaded_polygons.items()
            }

        if self.selected_room not in self.room_polygons:
            self.selected_room = next(iter(self.room_polygons), None)

        self.status_var.set(f"Loaded room polygons from {config_path}")
        self.publish_room_markers()

    def save_config(self):
        config_path = Path(self.config_path or _default_room_info_path())
        config_path.parent.mkdir(parents=True, exist_ok=True)

        if config_path.exists():
            backup_path = config_path.with_suffix(config_path.suffix + ".bak")
            shutil.copy2(config_path, backup_path)

        with self.lock:
            ordered_rooms = list(self.room_polygons.keys())
            config = {
                "room_polygons": {
                    _QuotedStr(room_name): [[float(x), float(y)] for x, y in self.room_polygons[room_name]]
                    for room_name in ordered_rooms
                }
            }

        with open(config_path, "w", encoding="utf-8") as file:
            yaml.dump(
                config,
                file,
                Dumper=_RoomYamlDumper,
                sort_keys=False,
                allow_unicode=True,
            )

        self.status_var.set(f"Saved room polygons to {config_path}")
        self.get_logger().info(f"Saved room polygons to {config_path}")
        self.publish_room_markers()

    def publish_room_markers(self):
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        timestamp = self.get_clock().now().to_msg()
        for room_index, (room_name, polygon) in enumerate(self.room_polygons.items()):
            color_r, color_g, color_b = _room_color(room_index)

            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = timestamp
            line_marker.ns = f"{room_name}_outline"
            line_marker.id = room_index * 10
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.05
            line_marker.color.r = color_r
            line_marker.color.g = color_g
            line_marker.color.b = color_b
            line_marker.color.a = 0.0
            for x, y in polygon:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.03
                line_marker.points.append(point)
            if polygon:
                first = Point()
                first.x = float(polygon[0][0])
                first.y = float(polygon[0][1])
                first.z = 0.03
                line_marker.points.append(first)
            marker_array.markers.append(line_marker)

            fill_marker = Marker()
            fill_marker.header.frame_id = "map"
            fill_marker.header.stamp = timestamp
            fill_marker.ns = f"{room_name}_fill"
            fill_marker.id = room_index * 10 + 1
            fill_marker.type = Marker.TRIANGLE_LIST
            fill_marker.action = Marker.ADD
            fill_marker.pose.orientation.w = 1.0
            fill_marker.scale.x = 1.0
            fill_marker.scale.y = 1.0
            fill_marker.scale.z = 1.0
            fill_marker.color.r = color_r
            fill_marker.color.g = color_g
            fill_marker.color.b = color_b
            fill_marker.color.a = 0.5
            if len(polygon) >= 3:
                for idx in range(1, len(polygon) - 1):
                    for px, py in [polygon[0], polygon[idx], polygon[idx + 1]]:
                        point = Point()
                        point.x = float(px)
                        point.y = float(py)
                        point.z = 0.01
                        fill_marker.points.append(point)
            marker_array.markers.append(fill_marker)

            point_marker = Marker()
            point_marker.header.frame_id = "map"
            point_marker.header.stamp = timestamp
            point_marker.ns = f"{room_name}_points"
            point_marker.id = room_index * 10 + 2
            point_marker.type = Marker.SPHERE_LIST
            point_marker.action = Marker.ADD
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.12
            point_marker.scale.y = 0.12
            point_marker.scale.z = 0.12
            point_marker.color.r = color_r
            point_marker.color.g = color_g
            point_marker.color.b = color_b
            point_marker.color.a = 1.0
            for x, y in polygon:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.05
                point_marker.points.append(point)
            marker_array.markers.append(point_marker)

        self.marker_pub.publish(marker_array)

    def refresh_room_list(self):
        self.room_listbox.delete(0, tk.END)
        for room_name in self.room_polygons.keys():
            self.room_listbox.insert(tk.END, room_name)

        if self.selected_room in self.room_polygons:
            index = list(self.room_polygons.keys()).index(self.selected_room)
            self.room_listbox.selection_set(index)
        elif self.room_polygons:
            self.selected_room = next(iter(self.room_polygons))
            self.room_listbox.selection_set(0)

        self.refresh_point_list()
        self.publish_room_markers()

    def refresh_point_list(self):
        self.point_listbox.delete(0, tk.END)
        if self.selected_room is None or self.selected_room not in self.room_polygons:
            return
        for index, point in enumerate(self.room_polygons[self.selected_room], start=1):
            self.point_listbox.insert(tk.END, f"{index}: ({point[0]:.3f}, {point[1]:.3f})")

    def on_room_selected(self, _event=None):
        selection = self.room_listbox.curselection()
        if not selection:
            return
        self.selected_room = self.room_listbox.get(selection[0])
        self.refresh_point_list()
        self.status_var.set(f"Selected room: {self.selected_room}")

    def prompt_for_text(self, title: str, label: str, initial_value: str = "", callback=None):
        dialog = tk.Toplevel(self.tk)
        dialog.title(title)
        dialog.geometry(f"420x120+{(self.width - 420) // 2}+{(self.height - 120) // 2}")

        tk.Label(dialog, text=label, font=("", 12)).place(x=20, y=20)
        entry = tk.Entry(dialog, width=28, font=("", 12))
        entry.insert(0, initial_value)
        entry.place(x=150, y=20)
        entry.focus_set()

        def submit():
            value = entry.get().strip()
            dialog.destroy()
            if callback:
                callback(value)

        tk.Button(dialog, width=10, text="Cancel", command=dialog.destroy).place(x=200, y=70)
        tk.Button(dialog, width=10, text="Set", command=submit).place(x=300, y=70)

    def add_room(self):
        def _add(room_name):
            normalized = _normalize_room_name(room_name)
            if not normalized:
                self.status_var.set("Room name cannot be empty.")
                return
            if normalized in self.room_polygons:
                self.status_var.set(f"Room already exists: {normalized}")
                return
            self.room_polygons[normalized] = []
            self.selected_room = normalized
            self.refresh_room_list()
            self.status_var.set(f"Added room: {normalized}")
            self.publish_room_markers()

        self.prompt_for_text("Add Room", "Room Name:", callback=_add)

    def rename_room(self):
        if self.selected_room is None:
            self.status_var.set("Select a room first.")
            return

        def _rename(room_name):
            normalized = _normalize_room_name(room_name)
            if not normalized:
                self.status_var.set("Room name cannot be empty.")
                return
            if normalized != self.selected_room and normalized in self.room_polygons:
                self.status_var.set(f"Room already exists: {normalized}")
                return

            points = self.room_polygons.pop(self.selected_room)
            self.room_polygons[normalized] = points
            self.selected_room = normalized
            self.refresh_room_list()
            self.status_var.set(f"Renamed room to: {normalized}")
            self.publish_room_markers()

        self.prompt_for_text("Rename Room", "Room Name:", self.selected_room, callback=_rename)

    def delete_room(self):
        if self.selected_room is None:
            self.status_var.set("Select a room first.")
            return
        room_name = self.selected_room
        del self.room_polygons[room_name]
        self.selected_room = next(iter(self.room_polygons), None)
        self.refresh_room_list()
        self.status_var.set(f"Deleted room: {room_name}")
        self.publish_room_markers()

    def append_point_to_selected_room(self, point, source="GUI"):
        if self.selected_room is None:
            self.status_var.set(f"Ignored {source} point because no room is selected.")
            return

        self.room_polygons.setdefault(self.selected_room, []).append(point)
        self.refresh_point_list()
        self.status_var.set(
            f"Added point to {self.selected_room} from {source}: ({point[0]:.3f}, {point[1]:.3f})"
        )
        self.publish_room_markers()

    def delete_point(self):
        if self.selected_room is None:
            self.status_var.set("Select a room first.")
            return
        selection = self.point_listbox.curselection()
        if not selection:
            self.status_var.set("Select a point first.")
            return
        del self.room_polygons[self.selected_room][selection[0]]
        self.refresh_point_list()
        self.status_var.set(f"Deleted point {selection[0] + 1} from {self.selected_room}")
        self.publish_room_markers()

    def move_point(self, direction: int):
        if self.selected_room is None:
            self.status_var.set("Select a room first.")
            return
        selection = self.point_listbox.curselection()
        if not selection:
            self.status_var.set("Select a point first.")
            return

        index = selection[0]
        new_index = index + direction
        points = self.room_polygons[self.selected_room]
        if new_index < 0 or new_index >= len(points):
            return

        points[index], points[new_index] = points[new_index], points[index]
        self.refresh_point_list()
        self.point_listbox.selection_set(new_index)
        self.publish_room_markers()

    def auto_order_points(self):
        if self.selected_room is None:
            self.status_var.set("Select a room first.")
            return

        points = self.room_polygons[self.selected_room]
        if len(points) < 3:
            self.status_var.set("At least 3 points are needed to order a polygon.")
            return

        self.room_polygons[self.selected_room] = _sort_points_clockwise(points)
        self.refresh_point_list()
        self.status_var.set(f"Auto-ordered points for {self.selected_room}")
        self.publish_room_markers()

    def clear_points(self):
        if self.selected_room is None:
            self.status_var.set("Select a room first.")
            return
        self.room_polygons[self.selected_room] = []
        self.refresh_point_list()
        self.status_var.set(f"Cleared all points for {self.selected_room}")
        self.publish_room_markers()

    def build_gui(self):
        geometry_x = 980
        geometry_y = 680
        self.tk.minsize(980, 640)
        self.tk.geometry(f"{geometry_x}x{geometry_y}+{(self.width - geometry_x) // 2}+40")

        left_frame = tk.Frame(self.tk)
        left_frame.place(x=20, y=20, width=370, height=520)
        tk.Label(left_frame, text="Rooms", font=("", 13)).pack(anchor="w")
        self.room_listbox = tk.Listbox(left_frame, exportselection=False, font=("", 12))
        self.room_listbox.pack(fill="both", expand=True, pady=8)
        self.room_listbox.bind("<<ListboxSelect>>", self.on_room_selected)

        right_frame = tk.Frame(self.tk)
        right_frame.place(x=420, y=20, width=540, height=520)
        tk.Label(right_frame, text="Polygon Points (map frame)", font=("", 13)).pack(anchor="w")
        self.point_listbox = tk.Listbox(right_frame, exportselection=False, font=("", 12))
        self.point_listbox.pack(fill="both", expand=True, pady=8)

        room_button_frame = tk.Frame(self.tk)
        room_button_frame.place(x=20, y=565, width=415, height=45)
        tk.Button(room_button_frame, width=12, text="Add Room", command=self.add_room).pack(side="left", padx=4)
        tk.Button(room_button_frame, width=12, text="Rename Room", command=self.rename_room).pack(side="left", padx=4)
        tk.Button(room_button_frame, width=12, text="Delete Room", command=self.delete_room).pack(side="left", padx=4)

        point_button_frame = tk.Frame(self.tk)
        point_button_frame.place(x=420, y=565, width=540, height=45)
        tk.Button(point_button_frame, width=12, text="Delete Point", command=self.delete_point).pack(side="left", padx=4)
        tk.Button(point_button_frame, width=8, text="Up", command=lambda: self.move_point(-1)).pack(side="left", padx=4)
        tk.Button(point_button_frame, width=8, text="Down", command=lambda: self.move_point(1)).pack(side="left", padx=4)
        tk.Button(point_button_frame, width=12, text="Auto Order", command=self.auto_order_points).pack(side="left", padx=4)
        tk.Button(point_button_frame, width=10, text="Clear", command=self.clear_points).pack(side="left", padx=4)

        control_frame = tk.Frame(self.tk)
        control_frame.place(x=20, y=620, width=600, height=40)
        tk.Button(control_frame, width=12, text="Reload", command=self.reload).pack(side="left", padx=4)
        tk.Button(control_frame, width=12, text="Save", command=self.save_config).pack(side="left", padx=4)
        tk.Button(control_frame, width=12, text="Close", command=self.close).pack(side="left", padx=4)

        tk.Label(self.tk, textvariable=self.status_var, anchor="w", justify="left").place(x=20, y=655, width=600)
        self.refresh_room_list()

    def reload(self):
        self.load_config()
        self.refresh_room_list()

    def close(self):
        self.tk.quit()
        self.tk.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = RoomPolygonSetting()

    configured_path = str(node.get_parameter("config_path").value).strip()
    node.config_path = configured_path or str(_default_room_info_path())

    try:
        node.select_config_file()
    except Exception as exc:
        node.get_logger().warning(f"Could not open file selector, using default room file path: {exc}")

    node.load_config()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    node.build_gui()
    node.tk.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
