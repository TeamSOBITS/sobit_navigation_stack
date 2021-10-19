# gmapping
gmappingはRBPF(Rao-Blackwellized ParticleFilter)-SLAMの代表例

RBPF-SLAMでは、再訪点検出を明示的に行わず、多数の仮設の中で再訪点の近くを通る仮説の評価値（尤度関数の値）が高くなり、  
パーティクルフィルタによって評価値の高いものが選択されることでループ閉じ込みを行います。  
しかし、屋外等の大規模な環境といった条件によっては、多くの仮説を生成しないと仮説が再訪点をうまく通らないことがあり，ループが閉じません。

## [sobit_turtlebot_gmapping.launch](/sobit_mapping/launch/sobit_turtlebot_gmapping.launch)
SOBIT EDU, MINI用の2次元地図生成(gmapping)
```bash
$ roslaunch sobit_mapping sobit_turtlebot_gmapping.launch 
```

## [sobit_pro_gmapping.launch](/sobit_mapping/launch/sobit_pro_gmapping.launch)
SOBIT PRO用の2次元地図生成(gmapping)
```bash
$ roslaunch sobit_mapping sobit_pro_gmapping.launch 
```

# 参考サイト
- [Google Cartographerとgmappingの比較](https://ssk0109.hatenablog.com/entry/2019/02/12/133340#gmapping)
- [ROS gmapping のパラメータ解説](https://sy-base.com/myrobotics/ros/gmapping/)
- [sobit turtlebot gmapping demo(YouTube)](https://www.youtube.com/watch?v=jon18pnzHeI)

---

- [目次に戻る](#sobit-mapping)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)

