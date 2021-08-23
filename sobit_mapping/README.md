# [SOBIT Mapping](/sobit_mapping)  
- 地図生成パッケージ

## sobit_turtlebot_gmapping.launch
SOBIT EDU, MINI用の地図生成(gmapping)

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
    - ロボットの移動は「キーボード」or「rviz上の2dnavGoal」
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

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)