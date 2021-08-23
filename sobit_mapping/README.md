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
- **使い方**
    - ロボットの現在地点が登録されます
    - 地点登録するには、端末で場所名を入力する
    - 登録を終了するには「q」を入力
- **注意点**
    - [sobit_turtlebot_gmapping.launch](/sobit_mapping/launch/sobit_turtlebot_create_location_file.launch)の「map_file」の指定を間違えない
    - Gazeboを使う場合は，[sobit_turtlebot_move_base.launch.xml](sobit_navigation/launch/include/sobit_turtlebot/sobit_turtlebot_move_base.launch.xml)の「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」から「mobile_base/commands/velocity」に変更してください。（GazeboではTopicに「cmd_vel_mux/input/navi」がないため）

```bash
$ roslaunch sobit_mapping sobit_turtlebot_create_location_file.launch 
```
- [sobit_turtlebot_create_location_file.launch](/sobit_mapping/launch/sobit_turtlebot_create_location_file.launch)
<div align="center">
    <img src="doc/img/sobit_turtlebot_create_location_file.png">
</div> 

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)