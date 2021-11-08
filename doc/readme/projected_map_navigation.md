# projected_map(立体的な障害物を押しつぶした2次元地図)を用いたナビゲーションをする方法
基本的には，通常の2次元地図でのナビゲーションと同じで，以下の2つのを行う

## 01. map_fileにprojected_mapを指定する
- [例:sobit_turtlebot_navigation_octomap.launch](sobit_navigation/launch/sobit_turtlebot_navigation_octomap.launch)
```xml
<!--  ファイル名を指定するなら下記のargを変更すること -->
<arg name="map_file" default="$(find sobit_mapping)/map/projected_map_8_26_19_54.yaml"/>
<arg name="initial_pose_x" default="0.0"/>
<arg name="initial_pose_y" default="0.0"/>
<arg name="initial_pose_a" default="0.0"/>
```

## 02. コストマップのパラメータを変更
1. observation_sourcesにLRFとRGB-Dセンサのデータを追加する
    - [例1：octomap_costmap_common_params.yaml](sobit_navigation/param/sobit_pro_octomap/octomap_costmap_common_params.yaml)
    ```xml
    observation_sources:  scan rgbd
    scan:
        data_type: LaserScan
        topic: scan
        marking: true
        clearing: true
        min_obstacle_height: 0.25
        max_obstacle_height: 0.35
    rgbd:
        data_type: PointCloud2
        topic: camera/depth/points
        marking: true
        clearing: false
        min_obstacle_height: 0.15
        max_obstacle_height: 1.0
    # for debugging only, let's you see the entire voxel grid
    ```

2.  orizin_zを変更
    グリッドの高さ(z_resolution * z_voxels)をmax_obstacle_heightと同じにする
     - [例1：octomap_costmap_common_params.yaml](sobit_navigation/param/sobit_pro_octomap/octomap_costmap_common_params.yaml)
    ```xml
    max_obstacle_height: 2.0
    origin_z:            0.0
    z_resolution:        0.2
    z_voxels:            10
     ```
     <div align="center">
    <img src="doc/img/fix_costmap.png">
    </div> 

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)