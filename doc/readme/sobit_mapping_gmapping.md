# gmapping
gmappingはRBPF(Rao-Blackwellized ParticleFilter)-SLAMの代表例

- [目次に戻る](../../sobit_mapping)

## 01. [sobit_turtlebot_gmapping.launch](../../sobit_mapping/launch/sobit_turtlebot/sobit_turtlebot_gmapping.launch)
SOBIT EDU, MINI用の2次元地図生成(gmapping)
```bash
$ roslaunch sobit_mapping sobit_turtlebot_gmapping.launch
# 引数
# rviz : Rvizを起動するか(bool)
# rqt_reconfigure : rqt_reconfigureを起動するか(bool)
# use_keyboard : キーボード操作をするか(bool)
```

## 02. [sobit_pro_gmapping.launch](../../sobit_mapping/launch/sobit_pro/sobit_pro_gmapping.launch)
SOBIT PRO用の2次元地図生成(gmapping)
```bash
$ roslaunch sobit_mapping sobit_pro_gmapping.launch
# 引数
# rviz : Rvizを起動するか(bool)
# rqt_reconfigure : rqt_reconfigureを起動するか(bool)
# use_keyboard : キーボード操作をするか(bool)
```

## 03. [sobit_turtlebot_gmapping_multi_merger.launch](../../sobit_mapping/launch/sobit_turtlebot/sobit_turtlebot_gmapping_multi_merger.launch)
SOBIT EDU, MINI用の2次元地図生成(gmapping)にRGB-Dセンサのデータが加わります
```bash
$ roslaunch sobit_mapping sobit_turtlebot_gmapping_multi_merger.launch
# 引数
# rviz : Rvizを起動するか(bool)
# rqt_reconfigure : rqt_reconfigureを起動するか(bool)
# use_keyboard : キーボード操作をするか(bool)
```

## 04. [sobit_pro_gmapping_multi_merger.launch](../../sobit_mapping/launch/sobit_pro/sobit_pro_gmapping_multi_merger.launch)
SOBIT PRO用の2次元地図生成(gmapping)にRGB-Dセンサのデータが加わります
```bash
$ roslaunch sobit_mapping sobit_pro_gmapping_multi_merger.launch
# 引数
# rviz : Rvizを起動するか(bool)
# rqt_reconfigure : rqt_reconfigureを起動するか(bool)
# use_keyboard : キーボード操作をするか(bool)
```

# 参考サイト
- [Google Cartographerとgmappingの比較](https://ssk0109.hatenablog.com/entry/2019/02/12/133340#gmapping)
- [ROS gmapping のパラメータ解説](https://sy-base.com/myrobotics/ros/gmapping/)
- [sobit turtlebot gmapping demo(YouTube)](https://www.youtube.com/watch?v=jon18pnzHeI)
- [How To Convert a PointCloud Into a Laser Scan](https://www.theconstructsim.com/ros-qa-120-how-to-convert-a-pointcloud-into-a-laser-scan/)
- [laserscan_multi_mergerで複数のスキャントピックを1つにマージする](https://rb-station.com/blogs/article/ros-laserscan_multi_merger)
---

- [目次に戻る](sobit_mapping)

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)

