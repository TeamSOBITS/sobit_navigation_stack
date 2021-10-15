# dwa_local_plannerのパラメータについて

---

## ロボット構成パラメータ
### 座標軸毎の最大加速度(m/s^2), 最大旋回加速度(rad/s^2)
- acc_lim_x: 2.5 #(double, default: 2.5)
- acc_lim_y: 2.5 #(double, default: 2.5) y軸方向に動けないロボットの場合は0にする
- acc_lim_theta: 3.2 #(double, defautl: 3.2)
- acc_lim_trans: 1.0 #(double, default: 1.0)

### 最大、最小並進速度(m/s)
- max_vel_trans: 0.55 #(double, default: 0.55) ロボットが直線上を移動するときの最大速度?
- min_vel_trans: 0.1  #(double, default: 0.1) 負の値を設定すると後退動作が可能になる。

### 座標軸毎の最大速度、最小速度(m/s)
- max_vel_x: 0.55 #(double, default: 0.55) x軸方向の移動速度=max_vel_transと同値にする
- min_vel_x: 0.0  #(double, default: 0.0) 負の値を設定すると後退動作が可能になる。
- max_vel_y: 0.1  #(double, default: 0.1) y軸方向に動けないロボットの場合は0にする
- min_vel_y: -0.1 #(double, default: -0.1) y軸方向に動けないロボットの場合は0にする

### 最大, 最小旋回速度(rad/s)
- max_vel_theta: 1.0 #(double, default: 1.0)
- min_vel_theta: 0.4 #(double, default: 0.4)

---

## 目標許容パラメータ
### 向きの許容誤差(rad)
- yaw_goal_tolerance: 0.05 #(double, default: 0.05)
- 目標に到達した時のロボットのyaw回転に置けるradの許容値 = 向きの許容誤差
  
### 位置の許容誤差(m)
- xy_goal_tolerance: 0.1 #(double, default: 0.10)
- 目標に到達した時のロボットのx,y方向の距離に置けるm単位の許容誤差 = 位置の許容誤差

### ロボットが停止しているとみなされる並進速度以下の速度
- trans_stopped_vel: 0.1 #(double, default: 0.1)
- theta_stopped_vel: 0.1 #(double, default: 0.1)

## フォワードシミュレーションパラメータ
### 軌跡をフォワードシミュレーションする時間(s)
- sim_time: 1.7  #(double, default: 1.7)

### 各軌道に沿って衝突をチェックする粒度(m)
- sim_granularity: 0.025  #(double, default: 0.025)

### 角速度のシミュレーションポイント間の距離
- angular_sim_granularity: 0.1 # (double, default:0.1)
- ロボットがロボットが物にぶつからない程度に小さくする必要がある

### x速度空間を探索するときに使用するサンプル数
- vx_samples: 3 #(integer, default: 3)

### y速度空間を探索するときに使用するサンプル数
- vy_samples: 10  #(integer, default: 10)
- max_vel_y=0だと無視される

### theta速度空間を探索するときに使用するサンプル数
- vth_samples: 20 #(integer, default: 20)

### このコントローラーが呼び出される周期(Hz).
- controller_frequency: 20.0 #(double, default: 20.0)
※ move_base経由でこのプランナーが使用される場合、move_baseのcontroller_frequencyが優先されるため未設定でもよい

---

## 軌道スコアリングパラメータ
  
### 各軌道のスコアリングに使用されるコスト関数の計算式
```
cost =
    path_distance_bias * (軌道の端点からパスまでの距離[m])
    + goal_distance_bias * (軌道の終点からローカルゴールまでの距離[m])
    + occdist_scale * (障害物コスト(0-254)の軌道に沿った最大障害物コスト)
    + twirling_scale * (軌道の旋回速度の絶対値)
``` 

### コントローラーが与えられたパスにどれだけ近づけるかの重み
- path_distance_bias: 0.6 #(double, default: 0.6)
- path_distance_biasを大きくするとよりグローバルパス近づく経路が生成される

### コントローラーがローカル目標を達成するためにどれだけの試行を行うべきかの重みづけ,また速度も制御する
- goal_distance_bias: 0.8  #(double, default: 0.8)
- goal_distance_biasを大きくするとよりローカルゴール（グローバルパスがローカルコストマップウィンドウの端から出て行く点）を目指す経路が生成される

### コントローラーがどれだけ障害物をよけようとするかの重み
- occdist_scale: 0.01 # (double, default: 0.01)
- occdist_scaleを大きくすると、障害物をより大きく回避するようなパスが生成される

### ロボットの方向の変更にペナルティを課すための重み
- twirling_scale: 0.0 #(double, default: 0.0)

### ロボットの中心点から追加のスコアリングポイントを配置するまでの距離(m)
- forward_point_distance: 0.325 #(double, default: 0.325)

### 軌道が有効とみなされるために、ロボットが衝突前に停止しなければならない時間を秒単位で表示する
- stop_time_buffer: 0.2 #(double, default: 0.2)

### ロボットのフットプリントのスケーリングを開始する速度の絶対値(m/s)
- scaling_speed: 0.25 #(double, default: 0.25)

### ロボットのフットプリントをスケーリングする最大係数
- max_scaling_factor: 0.2 #(double, default: 0.2)

### プランナーが計画時に使用するコストグリッドを公開するかどうか。
- publish_cost_grid_pc: false #(bool, default: false)

### 経路候補を描画するフレーム
- global_frame_id: "odom" #(string, default: "odom")

### 経路候補を配信するかどうか
- publish_traj_pc: false #(bool, default: false)

### 経路のコストを計算する際に距離をコストマップのセル数で計算する場合
- meter_scoreing: false #(bool, default: false) 
- 経路のコストを計算する際に距離をコストマップのセル数で計算するか、距離で計算するか。
```
cost =
    p_dist_scale * （グローバルパスまでの距離）
    + g_dist_scale * （ローカルゴールまでの距離）
    + occdist_scale * （通過する経路のセルの中でコスト（0~254）が最大のもの）
```


### コントローラーが与えられたパスにどれだけ近づけるかの重み
- p_dist_scale: 0.6 #(double, default 0.6) 
- p_distを大きくするとよりグローバルパス近づく経路が生成される

### コントローラーがローカル目標を達成するためにどれだけの試行を行うべきかの重みづけ,また速度も制御する
- g_dist_scale: 0.8 # (double, default 0.8) 
- g_dist_scaleを大きくするとよりローカルゴール（グローバルパスがローカルコストマップウィンドウの端から出て行く点）を目指す経路が生成される

### コントローラーがどれだけ障害物をよけようとするかの重み
- occdist_scale: 0.01 #(double, default 0.01) 
- occdist_scaleを大きくすると、障害物をより大きく回避するようなパスが生成される

### その場回転を評価するとき、どのくらい先をみるか
- heading_lookahead: 0.60 #(double [m], default: 0.325) 
- ロボットが前進できない時、その場回転を選択することがある。その時、どれくらい先を見てコストを評価するか。
- ロボットの少し先をスコアリングポイントとすることで、ロボットの向きを合わせることができる。

### ロボットの向きも考慮
- heading_scoring: true #(bool, default: false) 
- コストを評価するとき、ロボットの向きも考慮に入れるかどうか。

### 何秒先を見るか
- heading_scoring_timestep: 0.8 #(double [s], default: 0.8) 
- 何秒先を見るか。heading_scoringをtrueにした時に使用される。

---

## 発振防止パラメータ

### 発振リセット距離(m)
- oscillation_reset_dist: 0.05 #(double, default: 0.05)
- 発振フラグがリセットされるまでにロボットがどれくらいの距離を移動しなければならないか
- ロボットが振動したときにリカバリー行動する可能性があるので、どれだけ移動したら振動のカウンタをリセットするかの閾値

### 振動フラグがリセットされる前にロボットが回転しなければ行けない角度(rad)
- oscillation_rest_angle: 0.2   #(double, default: 0.2)

### ロボットが経路に沿って移動する際にプランを食べつくすかどうかを指定する。
- prune_plan: true #(bool, default: true)
- trueにすると、ロボットが1m移動するとポイントがプランの端から落ちていく
 
--- 

## その他
### DWAを使用するか
- use_dwa: true #(bool, default: true)
- ダイナミックウィンドウアプローチを使用してサンプリング速度を小さなウィンドウに制限する

### 元の構成に戻す
- restore_defaults: false #(bool default: false)

---

# 参考サイト
- [ROSのナビゲーションmove_baseについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-move_base/?amp)
- [DWA Plannerの設定可能パラメータ一覧](https://qiita.com/np_hsgw/items/ab3d4e34f4c1c160871d)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)