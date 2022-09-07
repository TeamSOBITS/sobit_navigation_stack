# 地点登録
OctoMapは，3次元占有グリッドマップを生成します

- [目次に戻る](sobit_mapping)

## 01. [sobit_turtlebot_create_location_file.launch](sobit_mapping/launch/sobit_turtlebot/sobit_turtlebot_create_location_file.launch)
SOBIT EDU, MINI用の地点登録(Gmapping)
- **使う前に**
    - [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```bash
    $ roslaunch sobit_mapping sobit_turtlebot_create_location_file.launch 
    ```

## 02. [sobit_turtlebot_create_location_file_multi_sensor.launch](sobit_mapping/launch/sobit_turtlebot/sobit_turtlebot_create_location_file_multi_sensor.launch)
SOBIT EDU, MINI用の地点登録(Octomap)
- **使う前に**
    - [sobit_turtlebot_navigation_multi_sensor.launch](sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_multi_sensor.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```bash
    $ roslaunch sobit_mapping sobit_turtlebot_create_location_file_multi_sensor.launch 
    ```

## 03. [sobit_pro_create_location_file.launch](sobit_mapping/launch/sobit_pro/sobit_pro_create_location_file_multi_sensor.launch)
SOBIT PRO用の地点登録(Gmapping)
- **使う前に**
    - [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```bash
    $ roslaunch sobit_mapping sobit_pro_create_location_file.launch 
    ```

## 04. [sobit_pro_create_location_file_multi_sensor.launch](sobit_mapping/launch/sobit_pro/sobit_pro_create_location_file_multi_sensor.launch)
SOBIT PRO用の地点登録(Octomap)
- **使う前に**
    - [sobit_pro_navigation_multi_sensor.launch](sobit_navigation/launch/sobit_pro/sobit_pro_navigation_multi_sensor.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```bash
    $ roslaunch sobit_mapping sobit_pro_create_location_file_multi_sensor.launch 
    ```

## How to Use
- ロボットの現在地点が登録されます
- 地点登録するには、端末で場所名を入力する
- 登録を終了するには「q」を入力
- ロボットの移動は「キーボード」or rviz上の「2D Nav Goal」

### 端末の表示例
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
    - 「map_file」の指定を間違えない
    - launch起動直後、端末に多くの情報が流れるため、「Location Name :」という表記が流されてしまうかもしれませんが、 気にせずに場所名を入力すれば地点登録されます

<div align="center">
    <img src="doc/img/sobit_turtlebot_create_location_file.png" width="640">
</div>

## 05. [location_file_viewer](sobit_mapping/launch/location_file_viewer.launch)
- ロボットを使用せずに、作成したロケーションファイルを確認するプログラム
- rviz上の「2D Nav Goal」で選択した位置をロケーションとして追加出来ます

- **使う前に**
    - [location_file_viewer.launch](sobit_mapping/launch/location_file_viewer.launch)の「map_file」と「location_file」のファイルパスを記入
    ```bash
    $ roslaunch sobit_mapping location_file_viewer.launch 
    ```

---

- [目次に戻る](sobit_mapping)

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)

