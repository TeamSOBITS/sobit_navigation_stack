# [SOBIT Navigation](/sobit_navigation) 
- 自律移動パッケージ 

## 01. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- **使う前に**
    - [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch rqt_reconfigure:=true
    # velocity_smootherなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch use_smoother:=false
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

## 02. [sobit_turtlebot_navigation_multi_sensor.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_multi_sensor.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(Octomapからの投影2次元地図(projected_map)を使用)
- **使う前に**
    - [sobit_turtlebot_navigation_multi_sensor.launch](sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_multi_sensor.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch rqt_reconfigure:=true
    # velocity_smootherなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch use_smoother:=false
    # カメラの向きの変更
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch use_pantilt_controll:=true use_sobit_mini:=false pan_angle_deg:=0.0 tilt_angle_deg:=10.0
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

## 03 [sobit_turtlebot_navigation_gmapping.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_gmapping.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- このとき地図生成も行う
    ```python
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_gmapping.launch
    ```

## 04. [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)
- SOBIT PRO用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- **使う前に**
    - [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_pro_navigation.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_pro_navigation.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_pro_navigation.launch rqt_reconfigure:=true
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

## 05. [sobit_pro_navigation_multi_sensor.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation_multi_sensor.launch)
- SOBIT PRO用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(Octomapからの投影2次元地図(projected_map)を使用)
- **使う前に**
    - [sobit_pro_navigation_multi_sensor.launch](sobit_navigation/launch/sobit_pro/sobit_pro_navigation_multi_sensor.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch rqt_reconfigure:=true
    # カメラの向きの変更
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch use_pantilt_controll:=true pan_angle_deg:=0.0 tilt_angle_deg:=10.0
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

## 06 [sobit_pro_navigation_gmapping.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation_gmapping.launch)
- SOBIT PRO用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- このとき地図生成も行う
    ```python
    $ roslaunch sobit_navigation sobit_pro_navigation_gmapping.launch
    ```

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)

