# 全方向移動するためのパラメータ設定
TurtleBotのナビゲーションパラメータでは，横方向移動ができないようになっています

そのため，SOBIT PROなどの全方向移動ロボットではパラメータを調整がしてあげる必要あります．

以下に、その調整手順を記述します

# 変更パラメータ
move_baseでは，経路生成としてA*とDWAが用いられます．

その経路生成に関わるパラメータを変更することで全方向移動を可能にしていきます．

## max_vel_y, min_vel_y
- 横方向に対する速度制限を設定するパラメータ
    - [例：dwa_local_planner_params.yaml](sobit_navigation/param/sobit_pro/dwa_local_planner_params.yaml)
    ```xml
    max_vel_y: 0.5
    min_vel_y: -0.5
    ```

## vy_samples
- DWAの横方向の速度の探索ステップ数を設定するパラメータ
    - [例：dwa_local_planner_params.yaml](sobit_navigation/param/sobit_pro/dwa_local_planner_params.yaml)
    ```xml
    vy_samples: 6
    ```

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)