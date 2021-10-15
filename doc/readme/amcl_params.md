# amclのパラメータについて

---

# 01. 総体的なフィルタ
- min_particles （ int型、デフォルト：100 ）
    - 粒子の最小許容数。

- max_particles （ int型、デフォルト：5000 ）
    - 粒子の最大許容数。

- kld_err （double型、デフォルト：0.01 ）
    - 真の分布と推定分布の間の最大誤差。

- kld_z （double型、デフォルト： 150 ）
    - pは推定distrubition上の誤差がkld_err未満になる確率ですが、 - の上限標準正規分位点（ P 1 ） 。

- update_min_d （double型、デフォルト： 0.2メートル）
    - フィルタのアップデートを実行する前に必要とされる並進運動。 

- update_min_a （double型、デフォルト： π/6.0ラジアン）
    - フィルタのアップデートを実行する前に必要とされる回転運動。

- resample_interval （ int型、デフォルト： 2）
    - パーティクルをリサンプルする間隔。
    - パーティクルフィルタでは、各パーティクルの尤度にある程度のばらつきが生じた時点でリサンプリングが行われる。（その際に、尤度が低いパーティクルが消去される。）

- transform_tolerance （double型、デフォルト： 0.1秒）
    - センサデータやtfなどのtimestampと現在時刻とのズレの許容範囲。センサデータの更新が無い場合には処理が停止してしまうのではないかと思われる。[sec]

- recovery_alpha_slow （double型、デフォルト：0.0 （無効） ）
    - ランダムポーズを追加することによって回復するときに決定する際に使用される低速の平均体重フィルタの指数関数的な減衰率。
    - ROS Wikiでの推奨値は0.001

- recovery_alpha_fast （double型、デフォルト：0.0 （無効） ）
    - ランダムポーズを追加することによって回復するときに決定する際に使用される高速の平均体重フィルタの指数関数的な減衰率。
    - ROS Wikiでの推奨値は0.1

- initial_pose_x （double型、デフォルト： 0.0メートル）
    - ロボットの初期姿勢(a)

- initial_pose_y （double型、デフォルト： 0.0メートル）
    - ロボットの初期姿勢(y)

- initial_pose_a （double型、デフォルト：0.0ラジアン）
    - ロボットの初期姿勢(yaw)

- initial_cov_xx （double型、デフォルト： 0.5× 0.5メートル）
    - ロボットの初期姿勢の分散(x)。値が大きくなるほどばらまくパーティクルの分布、角度が広がる
    - ガウス分布とフィルター初期化するために使用。 

- initial_cov_yy （double型、デフォルト： 0.5× 0.5メートル）
    - ロボットの初期姿勢の分散(y)。値が大きくなるほどばらまくパーティクルの分布、角度が広がる
    - ガウス分布とフィルター初期化するために使用。 

- initial_cov_aa （double型、デフォルト： （ π/12 ）*（ π/12 ）ラジアン）
    - ロボットの初期姿勢の分散(yaw*yaw)。値が大きくなるほどばらまくパーティクルの分布、角度が広がる
    - ガウス分布とフィルター初期化するために使用。 

- gui_publish_rate （double型、デフォルト： -1.0 Hz）
    - rviz等で可視化するためにpublishされるトピックの周波数。[Hz]

- save_pose_rate （double型、デフォルト： 0.5 Hz）
    - ガウシアン（平均（姿勢）と分散）をパラメータサーバに保存する周波数。[Hz] 

- use_map_topic （ブール値、デフォルト：false）
    - trueに設定すると、 AMCLはむしろそのマップを受信するサービスコールを作るよりも、マップのトピックにサブスクライブします。

- first_map_only （ブール値、デフォルト：false）
    - trueに設定すると、 AMCLだけむしろ新しいものが受信されるたびに更新するよりも、それがサブスクライブする最初のマップを使います。

---

# 02. レーザーモデル
- laser_min_range （double型、デフォルト： -1.0 ）
    - 最小スキャン範囲。-1.0はレーザの報告最小範囲を使用。 
- laser_max_range （double型、デフォルト： -1.0 ）
    - 最大スキャン範囲。-1.0 はレーザの報告最大範囲を使用。
- laser_max_beams （ int型、デフォルト：30 ）
    - フィルタの更新時に各スキャンでどのように多くの等間隔のビームが使用される。 
- laser_z_hit （double型、デフォルト：0.95 ）
    - モデルのz_hit部分の混合重み。 
- laser_z_short （double型、デフォルト： 0.1）
    - モデルのz_short部分の混合重み。
- laser_z_max （double型、デフォルト：0.05 ）
    - モデルのz_max部分の混合重み。
- laser_z_rand （double型、デフォルト：0.05 ）
    - モデルのz_rand部分の混合重み。
- laser_sigma_hit （double型、デフォルト： 0.2メートル）
    - ガウスモデルの標準偏差は、モデルのz_hit一部に使用した。
- laser_lambda_short （double型、デフォルト： 0.1）
    - モデルのz_short部分の指数関数的な減衰パラメータ。
- laser_likelihood_max_dist （double型、デフォルト： 2.0メートル）
    - 最大距離はlikelihood_fieldモデルで使用するために、地図上に障害物インフレを行うため。 
- laser_model_type （文字列、デフォルトは" likelihood_field " ）
    - どのモデルを使用すると、ビームまたはlikelihood_fieldどちらか。

---

# 03. オドメトリモデル
「差分」と「全方位」のモデルタイプがある

- odom_model_type（文字列、デフォルトは"diff"）
    - どのモデル、"diff(差分)"または "omni(全方位)"のいずれかを使用する。
 
- odom_alpha1（ダブル、デフォルト：0.2）
    - ロボットの動きの回転成分からオドメトリの自転の見積りの予想ノイズを指定します。
 

- odom_alpha2（ダブル、デフォルト：0.2）
    - ロボットの動きの並進成分からオドメトリの自転の見積りの予想ノイズを指定します。
 
- odom_alpha3（ダブル、デフォルト：0.2）
    - ロボットの動きの並進成分からオドメトリの翻訳上の見積りの予想されるノイズを指定します。
 
- odom_alpha4（ダブル、デフォルト：0.2）
    - ロボットの動きの回転成分からオドメトリの翻訳見積もりの予想ノイズを指定します。

- odom_alpha5（ダブル、デフォルト：0.2）
    - 翻訳関連のノイズパラメータ（モデルは"全方位"である場合にのみ使用）。

- odom_frame_id（文字列、デフォルトは"odom"）
    - どのフレームオドメトリに使用する。

- base_frame_id（文字列、デフォルトは"base_link"）
    - どのフレームロボットベースに使用する

- global_frame_id（文字列、デフォルトは"map"）
    - 座標系の名前はローカライゼーションシステムによって発行される

---

# 参考サイト
- [ROSのナビゲーションamclについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-amcl/)
- [amclパラメーター](http://rosrosrosrosros.blogspot.com/2013/09/amcl.html)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)