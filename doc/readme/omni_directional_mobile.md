# 全方向移動するためのパラメータ設定
TurtleBotのナビゲーションパラメータでは，横方向移動ができないようになっています

そのため，SOBIT PROなどの全方向移動ロボットではパラメータを調整がしてあげる必要あります．

以下に、その調整手順を記述します

# 変更パラメータ
move_baseでは，経路生成としてA*とDWAが用いられます．

その経路生成に関わるパラメータを変更することで全方向移動を可能にしていきます．

## max_vel_y, min_vel_y  (double [rad/s], default: 0.1, -0.1)
- 横方向に対する速度制限を設定するパラメータ
    - [例：dwa_local_planner_params.yaml](sobit_navigation/param/sobit_pro/dwa_local_planner_params.yaml)
    ```xml
    max_vel_y: 0.5
    min_vel_y: -0.5
    ```

## acc_lim_y (double [m/s^2], default: 2.5)
- 横方向の最大加速度。（全方位移動車両等のホロノミックロボットの場合にのみ使用）
    - [例：dwa_local_planner_params.yaml](sobit_navigation/param/sobit_pro/dwa_local_planner_params.yaml)
    ```xml
     acc_lim_y: 1.0
    ```

## vy_samples
- DWAの横方向の速度の探索ステップ数を設定するパラメータ
    - [例：dwa_local_planner_params.yaml](sobit_navigation/param/sobit_pro/dwa_local_planner_params.yaml)
    ```xml
    vy_samples: 6
    ```

## holonomic_robot (bool, default: true)
- ホロノミックなロボットかどうか。全方位移動車両の場合にはtrueにする。差動二輪やステアリング型の場合はfalse。  
（要するにロボットが真横に旋回せずに直接移動できるかどうか）
    - [例：dwa_local_planner_params.yaml](sobit_navigation/param/sobit_pro/dwa_local_planner_params.yaml)
    ```xml
    holonomic_robot: true
    ```

# 参考サイト
- [ROSのナビゲーションmove_baseについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-move_base/?amp)
- [DWA Plannerの設定可能パラメータ一覧](https://qiita.com/np_hsgw/items/ab3d4e34f4c1c160871d)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)