# [SOBIT Navigation](/sobit_navigation) 
- 自律移動パッケージ 

## 01. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot_navigation.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- **使う前に**
    1. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入
    2. Gazeboを使う場合
        - [sobit_turtlebot_move_base.launch.xml](sobit_navigation/launch/include/sobit_turtlebot/sobit_turtlebot_move_base.launch.xml)の「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」から「mobile_base/commands/velocity」に変更（GazeboではTopicに「cmd_vel_mux/input/navi」がないため）

    ```bash
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch 
    ```

## 02. [sobit_turtlebot_navigation_octomap.launch](/sobit_navigation/launch/sobit_turtlebot_navigation_octomap.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(Octomapからの投影2次元地図(projected_map)を使用)
- **使う前に**
    1. [sobit_turtlebot_navigation_octomap.launch](sobit_navigation/launch/sobit_turtlebot_navigation_octomap.launch)の「map_file」に地点登録するMapのファイルパスを記入
    2. Gazeboを使う場合
        - [sobit_turtlebot_move_base_octomap.launch.xml](sobit_navigation/launch/include/sobit_turtlebot_octomap/sobit_turtlebot_move_base_octomap.launch.xml)の「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」から「mobile_base/commands/velocity」に変更（GazeboではTopicに「cmd_vel_mux/input/navi」がないため）

    ```bash
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_octomap.launch 
    ```

## 03. [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro_navigation.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- **使う前に**
    1. [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```bash
    $ roslaunch sobit_navigation sobit_pro_navigation.launch 
    ```

## 04. [sobit_pro_navigation_octomap.launch](/sobit_navigation/launch/sobit_pro_navigation_octomap.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(Octomapからの投影2次元地図(projected_map)を使用)
- **使う前に**
    1. [sobit_pro_navigation_octomap.launch](sobit_navigation/launch/sobit_pro_navigation_octomap.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```bash
    $ roslaunch sobit_navigation sobit_pro_navigation_octomap.launch 
    ```





---




- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)

