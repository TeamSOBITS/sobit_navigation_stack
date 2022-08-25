# dwa_local_plannerのパラメータについて

---

## ロボット構成パラメータ
|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|acc_lim_x|double|2.50|m/s^2|ロボットのx軸方向加速度の上限|
|acc_lim_y|double|2.50|m/s^2|ロボットのy軸方向加速度の上限|
|acc_lim_theta|double|3.20|rad/s^2|ロボットの回転加速度の上限|
|acc_lim_trans|double|1.00|m/s^2|ロボットの並進運動加速度の上限<br>現在のソースでは無効なパラメーター|
|max_vel_trans|double|0.55|m/s|ロボットの並進運動速度の絶対値の上限<br>縦方向と横方向の合成速度の上限<br>これを超える軌道は破棄します．|
|min_vel_trans|double|0.10|m/s|ロボットの並進運動速度の絶対値の下限<br>縦方向と横方向の合成速度の下限<br>これと min_vel_theta のどちらも満たさない軌道は破棄します．|
|max_vel_x|double|0.55|m/s|ロボットのx軸方向速度の上限|
|min_vel_x|double|0.00|m/s|ロボットのx軸方向速度の下限<br>負の値を設定すると後退動作が可能になる．|
|max_vel_y|double|0.10|m/s|ロボットのy軸方向速度の上限<br>左方向は正の値<br>y軸方向に動けないロボットの場合は0にする．|
|min_vel_y|double|-0.10|m/s|ロボットのy軸方向速度の下限<br>右方向は負の値<br>y軸方向に動けないロボットの場合は0にする．|
|max_vel_theta|double|1.00|rad/s|ロボットの回転速度絶対値の上限|
|min_vel_theta|double|0.00|rad/s|ロボットの回転速度絶対値の下限<br>これと min_vel_trans のどちらも満たさない軌道は破棄します．|

---

## 目標許容誤差パラメータ
|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|yaw_goal_tolerance|double|0.05|rad|ゴール地点に到達したときの，ロボットの向き(回転角)の許容誤差|
|xy_goal_tolerance|double|0.10|m|ゴール地点に到達したときの，ロボットの2D平面上距離の許容誤差|
|latch_xy_goal_tolerance|bool|false|-|ゴール許容誤差ラッチフラグ<br>trueの場合，ロボットがゴール地点に到達すると，後はその場回転のみ行います．<br>回転の間にゴール許容誤差の範囲外になることもあります．<br>falseの場合は，範囲外に出たら通常の動作に戻ります．|
|trans_stopped_vel|double|0.10|m/s|最終補正にあたって停止したとみなすX-Y合成速度．停止後その場回転します|
|theta_stopped_vel|double|0.10|rad/s|最終補正にあたって停止したとみなす回転速度．停止後その場回転します|

---

## フォワードシミュレーションパラメータ
|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|sim_time|double|1.70|s|軌跡をフォワードシミュレーションする時間|
|sim_granularity|double|0.025|m|与えられた軌道上の点間のステップサイズ<br>各軌道に沿って衝突をチェックする粒度|
|angular_sim_granularity|double|0.10|rad|与えられた軌道上の角度サンプル間のステップサイズ|
|vx_samples|int|3|-|速度空間のx軸方向速度を探索するときに使用するサンプルの数|
|vy_samples|int|10|-|速度空間のy軸方向速度を探索するときに使用するサンプルの数|
|vth_samples|int|20|-|速度空間の回転速度を探索するときに使用するサンプルの数|
|controller_frequency|double|20.0|Hz|このコントローラーが呼び出される頻度<br>コントローラーの名前空間に設定されていない場合，searchParamを使用して親の名前空間からパラメーターを読み取ります．<br>すなわち，move_base とともに使用する場合は move_base の "controller_frequency"パラメーターを設定するだけでよく，このパラメーターを未設定のままにしておけます．|

---

## 軌道スコアリングパラメータ(距離)
### 各軌道のスコアリングに使用されるコスト関数の計算式

> *cost* =<br>
>   *path_distance_bias* * **(軌道の端点からパスまでの距離[m])**<br>
>   +　*goal_distance_bias* * **(軌道の終点からローカルゴールまでの距離[m])**<br>
>   +　*occdist_scale* * **(障害物コスト[0-254]の軌道に沿った最大障害物コスト)**<br>
>   +　*path_distance_bias* * **(グローバルパスへの向きのコスト[m])**<br>
>   +　*goal_distance_bias* * **(ローカルゴールへの向きのコスト[m])**<br>
>   +　*twirling_scale* * **(軌道の旋回速度の絶対値[rad/s])**

|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|path_distance_bias|double|32.0|1/m|コントローラーがパスにどれだけ近づこうとするかの重み<br>path_distance_biasを大きくするとよりグローバルパス近づく経路が生成される．|
|goal_distance_bias|double|24.0|1/m|コントローラーがローカルの目標にどれだけ近づこうとするかの重み<br>このパラメーターは速度も制御します．<br>goal_distance_biasを大きくするとよりローカルゴール（グローバルパスがローカルコストマップウィンドウの端から出て行く点）を目指す経路が生成される．|
|occdist_scale|double|0.01|-|コントローラーが障害物をどれだけ回避しようとするかの重み<br>occdist_scaleを大きくすると，障害物をより大きく回避するようなパスが生成される．|
|twirling_scale|double|0.00|s/rad|ロボットの方向の変更にペナルティを課すための重み|
|forward_point_distance|double|0.325|m|追加のスコアリングポイントを配置するためのロボット中心点からの距離<br>ロボットの向きの評価で使用します．<br>base_local_plannerの[heading_lookahead](https://robo-marc.github.io/navigation_documents/base_local_planner.html#trajectory-scoring-parameters-baselocalplanner)に相当します．|
|stop_time_buffer|double|0.20|s|軌道が有効と見なされるために，衝突前にロボットが停止しなければならない時間<br>現状のソースコードでは無効|
|scaling_speed|double|0.25|m/s|ロボットのfootprintのスケーリングを開始する速度の絶対値<br>現状のソースコードではスケーリングは行っていないため無効|
|max_scaling_factor|double|0.20|-|ロボットのfootprintをスケーリングする最大係数<br>現状のソースコードではスケーリングは行っていないため無効|
|publish_cost_grid|bool|false|-|プランナーが計画時に使用するコストグリッドを公開するかどうか<br>trueの場合， sensor_msgs/PointCloud2 が/cost_cloudトピックで利用可能になります．<br>各点群はコストグリッドを表し，個々のスコアリング関数コンポーネントのフィールドを持ちます．<br>また，スコアリングパラメーターを考慮に入れた各セルの全体的なコストを持ちます．|
|use_dwa|bool|true|-|Dynamic Window Approach(DWA)を使用するか、Trajectory Rolloutを使用するか<br>経験的にDWAで十分らしい|
|publish_traj_pc|bool|false|-|経路候補をを公開するかどうか|
|global_frame_id|string|odom|-|経路候補を描画するフレーム|

---

## 発振防止パラメータ
|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|oscillation_reset_dist|double|0.05|m|振動フラグがリセットされるまでにロボットが移動する必要がある距離<br>ロボットが振動したときにリカバリー行動する可能性があるので，どれだけ移動したら振動のカウンタをリセットするかの閾値|
|oscillation_rest_angle|double|0.20|rad|振動フラグがリセットされるまでにロボットが回転する必要がある角度|

## グローバルプランパラメータ
|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|prune_plan|bool|true|-|ロボットがパスに沿って移動するときにプランを「食べていくか」を定義します．<br>trueに設定されている場合、ロボットが移動した際に経路のうち現在位置から1メートル以上過去の点は消します．|

---

## その他
|パラメータ名|型|デフォルト値|単位|意味|
|---|---|---|---|---|
|restore_defaults|bool|false|-|元の構成に戻す|

# base_local_planner パッケージとの比較
base_local_planner と dwa_local_planner パッケージは両方とも DWAを使えますが，次のような違いがあります。([参考](https://answers.ros.org/question/10718/dwa_planner-vs-base_local_planner/))
- dwa_local_plannerは (縦, 横, 回転) 方向の速度制約をサポートしますが，base_local_plannerは (縦, 回転) 方向の速度制約のみをサポートします。
    - 横方向速度についてユーザーが指令できるのは，事前に指定する有効なy速度リストのみです。
    - そのため，ホロノミックまたは疑似ホロノミックなロボットには， dwa_local_plannerの方が速度空間をよりよくサンプリングできるため適しています。
- dwa_local_planner の方が，ソースコードが整理されています。
- 他にも細かな違いがあります。個別のロボットでは base_local_plannerの方が性能を発揮することもあるかもしれませんが，作者の一人はコードが整理されていることなどからまずdwa_local_plannerを使うことを推奨しています。
---

# 参考サイト
- [ROSのナビゲーションmove_baseについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-move_base/?amp)
- [DWA Plannerの設定可能パラメータ一覧](https://qiita.com/np_hsgw/items/ab3d4e34f4c1c160871d)
- [経路探索のパラメータの設定](http://dailyrobottechnology.blogspot.com/2014/12/812.html)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)