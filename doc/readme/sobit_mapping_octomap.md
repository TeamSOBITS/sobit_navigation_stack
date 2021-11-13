# Octomap
OctoMapは，3次元占有グリッドマップを生成します

- [目次に戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack/-/tree/main/sobit_mapping)

## 01. [sobit_turtlebot_octomap.launch](/sobit_mapping/launch/sobit_turtlebot/sobit_turtlebot_octomap.launch)
SOBIT EDU, MINI用の3次元地図生成(octomap)

```bash
roslaunch sobit_mapping sobit_turtlebot_octomap.launch 
```

## 02. [sobit_pro_octomap.launch](/sobit_mapping/launch/sobit_pro/sobit_pro_octomap.launch)
SOBIT PRO用の3次元地図生成(octomap)

```bash
roslaunch sobit_mapping sobit_pro_octomap.launch 
```

<div align="center">
    <img src="doc/img/sobit_turtlebot_octomap.jpg" width="640">
</div> 

### 各端末について
<div align="center">
    <img src="doc/img/sobit_turtlebot_octomap_terminal.png" width="640">
</div> 

1. rtabmap
    - 座標情報が付与された点群をOctoMapに付与するための処理を行う(rtabmapでも地図生成している)
2. octomap_server_node
    - rtabmapから送られてくる点群を用いてOctoMapを作成する
3. turtlebot_teleop_key
    - キーボードで入力した方向でロボットを移動させる
4. save_3dmap_command
    - 3次元地図や2次元地図を保存する
    - save_3dmap_commandの端末上で「Enterキー」を入力すると保存
5. projected_map_saver
    - 立体的な障害物を押しつぶした2次元地図を保存する
    - projected_map_saverの端末上で「sキー」を入力すると保存
<div align="center">
    <img src="doc/img/nomal_map_and_projected_map.jpg" width="640">
</div> 

# 参考サイト
- [Octomapで遊んでみた](https://qiita.com/ryu_software/items/d13a70aacfc6a71cacdb#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB)
- [Setup RTAB-Map on Your Robot!](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)
- [octomap_server](http://wiki.ros.org/octomap_server)
- [sobit turtlebot octomap demo(YouTube)](https://www.youtube.com/watch?v=32QMeLIP2Yo)

---

- [目次に戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack/-/tree/main/sobit_mapping)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)

