# [SOBIT Mapping](/sobit_mapping)  
- 地図生成パッケージ

## sobit_turtlebot_gmapping.launch
SOBIT EDU, MINI用の2次元地図生成(gmapping)

```bash
$ roslaunch sobit_mapping sobit_turtlebot_gmapping.launch 
```
- [sobit_turtlebot_gmapping.launch](/sobit_mapping/launch/sobit_turtlebot_gmapping.launch)
- [ROS gmapping のパラメータ解説](https://sy-base.com/myrobotics/ros/gmapping/)
- [sobit turtlebot gmapping demo(YouTube)](https://www.youtube.com/watch?v=jon18pnzHeI)

## sobit_turtlebot_create_location_file.launch
SOBIT EDU, MINI用の地点登録
- **使う前に**
    1. [sobit_turtlebot_gmapping.launch](/sobit_mapping/launch/sobit_turtlebot_create_location_file.launch)の「map_file」に地点登録するMapのファイルパスを記入
    2. Gazeboを使う場合
        - [sobit_turtlebot_move_base.launch.xml](sobit_navigation/launch/include/sobit_turtlebot/sobit_turtlebot_move_base.launch.xml)の「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」から「mobile_base/commands/velocity」に変更（GazeboではTopicに「cmd_vel_mux/input/navi」がないため）

- **使い方**
    - ロボットの現在地点が登録されます
    - 地点登録するには、端末で場所名を入力する
    - 登録を終了するには「q」を入力
    - ロボットの移動は「キーボード」or rviz上の「2D Nav Goal」
        - 端末の表示例
        ```bash
        ========================================
        [ 登録地点一覧 ]
            [ 1 ] : a
            [ 2 ] : b

        [ ロボットの現在地点を登録 ] 
        場所名を入力してください。「q」で終了。
        Location Name : c              

        現在地点を「c」で保存します。

        transform.getOrigine().x(): -2.74012
        transform.getOrigine().y(): 0.727012
        transform.getOrigine().z(): 0
        transform.getRotation().x(): 0
        transform.getRotation().y(): 0
        transform.getRotation().z(): 0.999528
        transform.getRotation().w(): 0.0307304
        「/home/sobits/catkin_ws/src/sobit_navigation_stack/sobit_mapping/map/map_location_8_23_20_15.yaml　」に追記完了。
        ========================================
        ```
- **注意点**
    - [sobit_turtlebot_gmapping.launch](/sobit_mapping/launch/sobit_turtlebot_create_location_file.launch)の「map_file」の指定を間違えない
    - launch起動直後、端末に多くの情報が流れるため、「Location Name :」という表記が流されてしまうかもしれませんが、 気にせずに場所名を入力すれば地点登録されます

```bash
$ roslaunch sobit_mapping sobit_turtlebot_create_location_file.launch 
```
- [sobit_turtlebot_create_location_file.launch](/sobit_mapping/launch/sobit_turtlebot_create_location_file.launch)
<div align="center">
    <img src="doc/img/sobit_turtlebot_create_location_file.png">
</div> 

## sobit_turtlebot_octomap
SOBIT EDU, MINI用の3次元地図生成(octomap)

```bash
roslaunch sobit_mapping sobit_turtlebot_octomap.launch 
```
- [sobit_turtlebot_octomap.launch](/sobit_mapping/launch/sobit_turtlebot_octomap.launch)
- [Octomapで遊んでみた](https://qiita.com/ryu_software/items/d13a70aacfc6a71cacdb#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB)
- [Setup RTAB-Map on Your Robot!](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)
- [octomap_server](http://wiki.ros.org/octomap_server)
- [sobit turtlebot octomap demo(YouTube)](https://www.youtube.com/watch?v=32QMeLIP2Yo)

<div align="center">
    <img src="doc/img/sobit_turtlebot_octomap.jpg">
</div> 

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)