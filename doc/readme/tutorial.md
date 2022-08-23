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
$ git clone https://gitlab.com/TeamSOBITS/sobit_education_gazebo.git
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

<details><summary>turtlebot_teleop</summary>

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

<details><summary>save_map_command</summary>

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
地点登録を行う前に，次の2つを行ってください．
1. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMap(yaml)のファイルパスを記入
2. [sobit_turtlebot_move_base.launch.xml](sobit_navigation/launch/include/sobit_turtlebot/sobit_turtlebot_move_base.launch.xml)の「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」から「mobile_base/commands/velocity」に変更（GazeboではTopicに「cmd_vel_mux/input/navi」がないため）  
※逆に，SOBIT EDU & MINIを使う場合は，「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」に戻すこと．

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
- rviz上の「2D Nav Goal」で選択した位置をロケーションとして追加出来ます

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

地点登録を行う前に，次の2つを行ってください．
1. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMap(yaml)のファイルパスを記入
2. [sobit_turtlebot_move_base.launch.xml](sobit_navigation/launch/include/sobit_turtlebot/sobit_turtlebot_move_base.launch.xml)の「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」から「mobile_base/commands/velocity」に変更（GazeboではTopicに「cmd_vel_mux/input/navi」がないため）  
※逆に，SOBIT EDU & MINIを使う場合は，「cmd_vel」を設定を「navigation_velocity_smoother/raw_cmd_vel」に戻すこと．

sobit_turtlebot_navigationを起動
```bash
$ roslaunch sobit_navigation sobit_turtlebot_navigation.launch 
```
※sobit_education_gazebo.launchも起動してください

navigationを起動すると，Rvizと1つの端末(背景青)が起動します．
詳細は，それぞれをクリックすると見ることができます．

<details><summary>rviz</summary>

生成した地図を可視化することができます．
<div align="center">
    <img src="doc/img/navigation_rviz.png" width="1080">
</div>
</details>

# SOBIT Navigation Library
- actionlibを用いてソースコード上でナビゲーションを行えるライブラリです．  
- 基本的には，ロケーションファイルで登録した位置にナビゲーションするときに使用します．  
- 詳細は[こちら](sobit_navigation_library)
