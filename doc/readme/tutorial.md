# Tutorial
このチュートリアルでは，SOBIT Navigation Stackの使い方を学び、地図生成、地点登録、ナビゲーションを行います．

## 環境
|項目|値|
|---|---|
|Ubuntu|20.04|
|ROS|Noetic|
|Gazebo|11.9.0|

## 前提知識
1. ROSのTopic通信に関する知識
2. ROSのパラメータに関する知識
3. ROSのLaunchファイルに関する知識

## 目次
1. [Before Tutorial](#before-tutorial)
2. [SOBIT Navigation Stack](#sobit-navigation-stack)
3. [Mapping](#mapping)
4. [Location Registration](#location-registration)
5. [Navigation](#navigation)
6. [SOBIT Navigation Library](#sobit-navigation-library)

# Before Tutorial
実際のロボットを用意することなくTutorialを進めるため，SOBIT EDU Gazeboを追加します．
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/TeamSOBITS/sobit_education_gazebo.git
$ cd ~/catkin_ws/src/sobit_education_gazebo/
$ bash install.sh
```
install.shが完了したら，試しに起動してみましょう．(初回の起動には数分間程度の時間がかかります．)  
下の図のように，Gazeboが起動出来たら成功です．
```bash
# robocup_choi.world
$ roslaunch sobit_education_gazebo sobit_education_gazebo.launch world_name:=robocup_choi.world
```
<details><summary>sobit_education_gazebo</summary>

<div align="center">
    <img src="doc/img/sobit_edu_gazebo.png" width="640">
</div>

</details>

# SOBIT Navigation Stack
SOBIT Navigation Stackには，次の3つのパッケージで構成されています．
1. [sobit_mapping](sobit_mapping)
    - SOBIT EDU, SOBIT MINI, SOBIT PROで地図生成をするためのLaunchファイルとパラメータファイルをまとまたパッケージ
2. [sobit_navigation](sobit_navigation)
    - SOBIT EDU, SOBIT MINI, SOBIT PROでナビゲーションをするためのLaunchファイルとパラメータファイルをまとまたパッケージ
3. [sobit_navigation_library](sobit_navigation_library)
    - actionlibを用いてソースコード上でナビゲーションを行えるライブラリ

# Mapping
sobit_mappingパッケージを用いて地図生成をします．  
sobit_mappingパッケージでは，
- [2次元地図生成(gmapping)](doc/readme/sobit_mapping_gmapping.md)
- [3次元地図生成(octomap)](doc/readme/sobit_mapping_octomap.md)

の2種類の地図をSOBIT EDU, SOBIT MINI, SOBIT PROで生成することができます．
それぞれの地図の詳細は，上のリンクを参照してみてください．

今回は，SOBIT EDU Gazeboを使って，2次元地図を生成していきましょう．  
まず，Gazeboを起動します
```bash
$ roslaunch sobit_education_gazebo sobit_education_gazebo.launch world_name:=robocup_choi.world
```
次に，gmappingを起動します．
```bash
$ roslaunch sobit_mapping sobit_turtlebot_gmapping.launch 
```
gmappingを起動すると，Rvizと2つの端末(背景青)が起動します．
詳細は，それぞれをクリックすると見ることができます．

<details><summary>rviz</summary>

生成した地図を可視化することができます．
<div align="center">
    <img src="doc/img/mapping_rviz.png" width="1080">
</div>
</details>

<details><summary>turtlebot_teleop(xterm端末)</summary>

ロボットをキーボード操作し，地図を拡大することができます．
```bash
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit

currently:      speed 0.2       turn 1
```

</details>

<details><summary>save_map_command(xterm端末)</summary>

生成した地図をファイル形式に保存することができます．  
この端末上でEnter keyを押すことで地図を保存します．  
保存した地図は，「sobit_mapping/map/」に，pgmとして保存され，地図の原点やサイズ情報がyamlとして保存されます．  
ファイル名は，保存した日時となります．
```bash
When you save this map, press the 'Enter' key! : 
```
</details>

# Location Registration
生成した地図に対して，地点登録をします．  
登録した地点は，yamlファイルに記録されます．  
地点登録のLaunchファイルは，[こちら](doc/readme/sobit_mapping_create_location_file.md)を参照してください．  
今回は，SOBIT EDU Gazeboを使って，2次元地図に地点登録を行います．

地点登録には，「**create_location_file**」と「**location_file_viewer**」の2つの方法があります．  
create_location_fileは，実際にロボットを動かして，ロボットの現在位置に対して地点登録する方法です．  
この方法は，直感的で分かりやすいですが，オドメトリの誤差で登録地点もズレが生じてしまいます．  
そこでlocation_file_viewerで，誤差を修正します．  
rviz上の「2D Nav Goal」で選択した位置をロケーションとして追加できます．  
また，地点登録したyamlファイルを直接修正し，location_file_viewerで確認する方法もあります．  
個人的には，地図の原点が分かっている場合，登録地点の座標位置をyamlファイルに手打ちするのが楽でオススメです．

## create_location_file
地点登録を行う前に，次のことを行ってください．
1. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMap(yaml)のファイルパスを記入

create_location_fileを起動
```bash
$ roslaunch sobit_mapping sobit_turtlebot_create_location_file.launch 
```
※sobit_education_gazebo.launchも起動してください

### How to Use
- ロボットの現在地点が登録されます
- 地点登録するには、端末で場所名を入力する
- 登録を終了するには「q」を入力
- ロボットの移動は「キーボード」or rviz上の「2D Nav Goal」
- 保存したロケーションファイルは，「sobit_mapping/map/」に，yamlとして保存されます．  
- ファイル名は，保存した日時となります．

<details><summary>端末の操作方法</summary>

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

</details>

## location_file_viewer
- ロボットを使用せずに、作成したロケーションファイルを確認するプログラム
- rviz上の「2D Nav Goal」で選択した位置をロケーションとして追加できます

地点登録を行う前に，次のことを行ってください．
1. [location_file_viewer.launch](sobit_mapping/launch/location_file_viewer.launch)の「map_file(yaml)」と「location_file(yaml)」のファイルパスを記入

location_file_viewerを起動
```bash
$ roslaunch sobit_mapping location_file_viewer.launch 
```
※sobit_education_gazebo.launchも起動してください
※端末の操作方法は，create_location_fileと同じです．

# Navigation
sobit_navigationパッケージを用いてナビゲーションをします．  
今回は，SOBIT EDU Gazeboを使って，2次元地図を用いてをナビゲーションを行います．

地点登録を行う前に，次のことを行ってください．
1. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMap(yaml)のファイルパスを記入

sobit_turtlebot_navigationを起動
```bash
# velocity_smootherなし & rqt_reconfigureによるパラメータ調整
$ roslaunch sobit_navigation sobit_turtlebot_navigation.launch use_smoother:=false rqt_reconfigure:=true 
```
※sobit_education_gazebo.launchも起動してください  
※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

navigationを起動すると，Rvizと1つの端末(背景青)が起動します．  
rviz上の「2D Nav Goal」を使うことで目的位置を与えることができます．  
詳細は，それぞれをクリックすると見ることができます．

<details><summary>rviz</summary>

生成した地図や自己位置推定、ナビゲーションを可視化することができます．
<div align="center">
    <img src="doc/img/navigation_rviz.png" width="1080">
</div>
</details>

<details><summary>move_base(xterm端末)</summary>

```bash
[ INFO] [1661330610.382809644, 232.777000000]: global_costmap: Using plugin "static_layer"
[ INFO] [1661330610.388077331, 232.777000000]: Requesting the map...
[ INFO] [1661330610.582065826, 232.977000000]: Resizing costmap to 640 X 544 at 0.050000 m/pix
[ INFO] [1661330610.681121262, 233.077000000]: Received a 640 X 544 map at 0.050000 m/pix
[ INFO] [1661330610.683396674, 233.077000000]: global_costmap: Using plugin "obstacle_layer"
[ INFO] [1661330610.685693889, 233.077000000]:     Subscribed to Topics: scan bump
[ INFO] [1661330610.711716746, 233.107000000]: global_costmap: Using plugin "inflation_layer"
[ INFO] [1661330610.753926942, 233.147000000]: local_costmap: Using plugin "obstacle_layer"
[ INFO] [1661330610.756230563, 233.147000000]:     Subscribed to Topics: scan bump
[ INFO] [1661330610.785943128, 233.177000000]: local_costmap: Using plugin "inflation_layer"
[ INFO] [1661330610.815726845, 233.207000000]: Created local_planner dwa_local_planner/DWAPlannerROS
[ INFO] [1661330610.818887272, 233.207000000]: Sim period is set to 0.20
[ INFO] [1661330611.818819035, 234.207000000]: Recovery behavior will clear layer 'obstacle_layer'
[ INFO] [1661330611.820430353, 234.207000000]: Recovery behavior will clear layer 'obstacle_layer'
[ INFO] [1661330611.821990068, 234.207000000]: Recovery behavior will clear layer 'obstacle_layer'
[ INFO] [1661330611.826308800, 234.217000000]: Recovery behavior will clear layer 'obstacle_layer'
[ INFO] [1661330611.828127239, 234.217000000]: Recovery behavior will clear layer 'obstacle_layer'
[ INFO] [1661330611.829663739, 234.217000000]: Recovery behavior will clear layer 'obstacle_layer'
[ INFO] [1661330611.862336691, 234.247000000]: odom received!
[ INFO] [1661330616.925214506, 239.317000000]: Got new plan
[ INFO] [1661330617.923257379, 240.317000000]: Got new plan
[ INFO] [1661330618.921831119, 241.317000000]: Got new plan
[ INFO] [1661330619.921545958, 242.317000000]: Got new plan
[ INFO] [1661330620.924790276, 243.317000000]: Got new plan
                ・
                ・
                ・
```
</details>

## Parameter Turning
今回使用したGazebo環境では，目の前のコストマップが大きく部屋の中に入ることができません．  
また，SOBIT EDUの速度が大きく倒れてしまうことがあります  
そこで必要なのが，Parameter Turningです．  
以下のリンクを参照して，適切なパラメータをrqt_reconfigureを使用して，調整してみてください．

- [dwa_local_plannerのパラメータについて](doc/readme/dwa_params.md)
- [amclのパラメータについて](doc/readme/amcl_params.md)
- [recovery_behaviorsについて](doc/readme/recovery_behaviors.md)
- [コストマップに付与されるコストを変える](doc/readme/costmap_parameter_turning.md)

# SOBIT Navigation Library
- actionlibを用いてソースコード上でナビゲーションを行えるライブラリです．  
- 基本的には，ロケーションファイルで登録した位置にナビゲーションするときに使用します．  
- 詳細は[こちら](sobit_navigation_library)
