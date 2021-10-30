# recovery_behaviors
動作がスタックしたときに発動されるのが Recovery 機能

## move_slow_and_clear
- 障害物の近くでの速度を制限して移動し、移動できた場所の周りの障害物を地図から除去します
    ```
    1. ~<name> / clearing_distance（double、デフォルト：0.5）
        - 障害物を除去するためのロボットからの距離をメートル単位で指定します。

    2. ~<name> / limited_trans_speed（double，デフォルト：0.25）
        - move_slow_and_clearを実行している間、ロボットが制限される並進速度（メートル／秒）。

    3. ~<name> / limited_rot_speed（double、デフォルト：0.25）
        - move_slow_and_clearを実行する際にロボットが制限される回転速度（ラジアン/秒）を指定する。

    4. ~<name> / limited_distance（double、デフォルト：0.3）
    - 速度制限が解除されるまでにロボットが移動しなければならない距離をメートル単位で指定します。

    5. ~<name> / planner_namespace（文字列、デフォルト："DWAPlannerROS"）
        - パラメータを再設定するプランナーの名前です。具体的にはmax_trans_velとmax_rot_velのパラメータはこの名前空間内で再設定される。
    ```
## clear_costmap_recovery 
- 自分の周り半径 X m 以内の local costmap をクリアします
    ```
    1. 〜<name> / reset_distance（double、デフォルト：3.0）
        - ロボットのポーズを中心とする正方形の辺の長さ。その外側では、障害物が静的マップに戻されたときにコストマップから削除されます。 
    ```

## rotate_recovery
- その場で360度回転することで、 local costmap を更新します
    ```
    1. TrajectoryPlannerROS/yaw_goal_tolerance（ダブル、デフォルト：0.05）
        - 目標を達成したときのコントローラのヨー/ローテーションのラジアン単位の許容値

    2. TrajectoryPlannerROS/acc_lim_th（double、デフォルト：3.2）
        - ラジアン/秒^2で表したロボットの回転加速度の限界値

    3. TrajectoryPlannerROS/max_rotational_vel（double、デフォルト：1.0）
        - ベースに許容される回転速度の上限（単位：ラジアン/秒

    4. TrajectoryPlannerROS/min_in_place_rotational_vel（double、デフォルト：0.4）
        - その場での回転を行う際にベースに許容される最小回転速度をラジアン/秒で指定します。
    ```

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)