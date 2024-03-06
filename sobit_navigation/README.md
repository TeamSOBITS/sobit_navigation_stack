# Navigationをいろいろ工夫する
人間なら簡単に移動できるような環境でも，ロボットがNavigationをするにはまだまだ難易度が高い． \
<!-- そのため，どうしても人間のようにそのときそのときの状況に合わせた -->
そこでSOBITSのNavigationでは以下の2つのアプローチがある．

<details>
  <summary>アプローチ一覧</summary>
  <ol>
    <li>
      <a href="#障害物のレイヤーをカスタムする">概要</a>
    </li>
    <li>
      <a href="#重みパラメータを変更する">概要</a>
    </li>
  </ol>
</details>

## 障害物のレイヤーをカスタムする
Navigationでは，障害物の検出方法ごとに，レイヤーと呼ばれる層を設けている．\
通常2D LiDARにより検出された障害物は"scan"という層で，障害物のコストとして扱われ，回避を行うことができる． \
しかし，椅子やテーブルのような立体的に捉える必要のある障害物や，反対に2D LiDARよりも低い位置にある障害物を検出できない．
そこで3D cameraを使うことで，対応できるようにする． \
2D LiDARで取得できなかった障害物を元に，カメラから得た別のレイヤーとして設けることで，その障害物を避けることができる． \
ここでは，いろいろな障害物の検出方法から，いくつかの層について説明をする． \
また，そのレイヤーの使い方についても説明する．

1. SOBITSが歴代で作ってきたレイヤー
- scanレイヤー
2D LiDARによって得られた比較的広範囲の障害物をコストとして捉えることができるレイヤー．\
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| 水平方向に広範囲 | 固定の高さしか検出できない |
| 処理が早い | 薄い情報量 |

- bumperレイヤー
turtlebotの前方にあるバンパーが押されたときに停止するシステム．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| LiDARよりも低い物体を検知 | turtlebotにしかない |
|  | 衝突しないと検出しない |

- obstacleレイヤー
3D カメラから得られる全ての点群データから，床面を除去した凸部分をコストとする．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| LiDARよりも低い物体を検知 | 使用するカメラの点群の精度に依存する |
| 立体的な物体を検出できる | 小さすぎる物体は床と判定されて回避できない |

- noise_colorレイヤー
全ての点群データから，床面と近い色を除去して，色が違う部分をコストとする．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| 物体の大きさに依存しない | 使用するカメラの点群の精度に依存する |
| 床と色が違えば高さによらず検出できる | 床の色が単色のときにしか使えない |

- objectsレイヤー
画像から物体を検出し，その位置の点群をコストとする．\
例えば，yoloで検出した物体を回避することで，物体の大きさに依存しない画像基準の検出での回避ができる．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| 物体の大きさに依存しない | 画像のピクセル数や点群の処理速度に依存 |
| 床の柄が複雑でも適切な検出が可能 | 学習している物体しか検出できない |


2. レイヤーを設定する
    上のレイヤーを組み合わせてNavigationをすることができる．\
    レイヤー設定のファイルのobservation_sourcesへ，設定したいレイヤーを指定する．
    - SOBIT PRO用のレイヤー設定のファイル \
        [/sobit_navigation/param/sobit_pro/costmap_common_params.yaml](/sobit_navigation/param/sobit_pro/costmap_common_params.yaml)のobservation_sourcesを書き換える．
    - SOBIT EDU，SOBIT MINI用のレイヤー設定のファイル \
        [/sobit_navigation/param/sobit_turtlebot/costmap_common_params.yaml](/sobit_navigation/param/sobit_turtlebot/costmap_common_params.yaml)のobservation_sourcesを書き換える．
    observation_sourcesは「#」で以降コメントアウトで，そのコメントアウトされているもの中から選んでコメントアウトを外すことでカスタムできる．\
    以下は，レイヤーのうちscanとnoise_colorのレイヤーを選んだ例．
    ```xml
    observation_sources: scan noise_color    #scan #bumper #obstacle #noise_color #objects
    ```


3. 3D カメラの設定をする
    レイヤーが3D カメラに依存するものの場合は，カメラを設定する．
    - SOBIT PROでナビゲーション
        [/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)のuse_3d_camera，pan_angle_deg，tilt_angle_deg，cloud_topic_nameを書き換える．\
        use_3d_cameraはtrueにする．\
        pan_angle_deg，tilt_angle_degはカメラをどれだけ回転させるかをパン・チルト回転で表記する．\
        cloud_topic_nameは使用する3D カメラの点群(sensor_msgs/PointCloud2)が格納されているものを指定する．
        例えばazure kinectならば，"/points2"である．\
        以下は，SOBIT PROの3D カメラ(azure kinect)を用いて，カメラを左右に0度，下に40度変えてNavigationする場合の例
        ```xml
        <arg name="use_3d_camera"    default="true" />
        <arg name="pan_angle_deg"    default="0.0"/>
        <arg name="tilt_angle_deg"   default="-40.0"/>
        <arg name="cloud_topic_name" default="/points2"/>
        ```
    - SOBIT EDU，SOBIT MINIでナビゲーション
        [/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)のuse_3d_camera，use_sobit_，pan_angle_deg，tilt_angle_deg，cloud_topic_nameを書き換える．\
        use_3d_cameraはtrueにする．\
        use_sobit_はSOBIT EDUを使う場合はedu，SOBIT MINIを使う場合はminiとする．\
        pan_angle_deg，tilt_angle_degはカメラをどれだけ回転させるかをパン・チルト回転で表記する．\
        cloud_topic_nameは使用する3D カメラの点群(sensor_msgs/PointCloud2)が格納されているものを指定する．
        例えばazure kinectならば，"/points2"である．\
        以下は，SOBIT MINIの3D カメラ(realsense)を用いて，カメラを左右に0度，下に35度変えてNavigationする場合の例
        ```xml
        <arg name="use_3d_camera"                 default="true"/>
        <arg name="use_sobit_"                    default="mini"  />   <!-- edu or mini -->
        <arg name="pan_angle_deg"                 default="0.0"  />
        <arg name="tilt_angle_deg"                default="-35.0"  />
        <arg name="cloud_topic_name"              default="/head_camera/depth_registered/points"/>  <!-- azure_kinect="/points2", realsense="/head_camera/depth_registered/points" -->
        ```

4. ロボットの起動とNavigationの起動
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．\
    以下のコマンドで通常通りNavigationを起動する． 
    - SOBIT PROでナビゲーション
    ```sh
    $ roslaunch sobit_navigation sobit_pro_navigation.launch
    ```
    - SOBIT EDU，SOBIT MINIでナビゲーション
    ```sh
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch
    ```

## 重みパラメータを変更する


<!-- -----------------------------以下修正前のもの----------------------------- -->



<!-- # [SOBIT Navigation](/sobit_navigation) 
- 自律移動パッケージ 

## 01. [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- **使う前に**
    - [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch rqt_reconfigure:=true
    # velocity_smootherなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch use_smoother:=false
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。   -->

<!-- ## 02. [sobit_turtlebot_navigation_multi_sensor.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_multi_sensor.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(Octomapからの投影2次元地図(projected_map)を使用)
- **使う前に**
    - [sobit_turtlebot_navigation_multi_sensor.launch](sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_multi_sensor.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch rqt_reconfigure:=true
    # velocity_smootherなし
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch use_smoother:=false
    # カメラの向きの変更
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_multi_sensor.launch use_pantilt_controll:=true use_sobit_mini:=false pan_angle_deg:=0.0 tilt_angle_deg:=10.0
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。   -->

<!-- ## 03 [sobit_turtlebot_navigation_gmapping.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation_gmapping.launch)
- SOBIT EDU, MINI用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- このとき地図生成も行う
    ```python
    $ roslaunch sobit_navigation sobit_turtlebot_navigation_gmapping.launch
    ``` -->

<!-- ## 04. [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)
- SOBIT PRO用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- **使う前に**
    - [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_pro_navigation.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_pro_navigation.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_pro_navigation.launch rqt_reconfigure:=true
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。   -->

<!-- ## 05. [sobit_pro_navigation_multi_sensor.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation_multi_sensor.launch)
- SOBIT PRO用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(Octomapからの投影2次元地図(projected_map)を使用)
- **使う前に**
    - [sobit_pro_navigation_multi_sensor.launch](sobit_navigation/launch/sobit_pro/sobit_pro_navigation_multi_sensor.launch)の「map_file」に地点登録するMapのファイルパスを記入

    ```python
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch
    # rvizなし
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch rviz:=false
    # rqt_reconfigureによるパラメータ調整
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch rqt_reconfigure:=true
    # カメラの向きの変更
    $ roslaunch sobit_navigation sobit_pro_navigation_multi_sensor.launch use_pantilt_controll:=true pan_angle_deg:=0.0 tilt_angle_deg:=10.0
    ```
    ※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
    ※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。   -->

<!-- ## 06 [sobit_pro_navigation_gmapping.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation_gmapping.launch)
- SOBIT PRO用のrviz上の「2D Nav Goal」などから目的地を与え、そこまで自律移動する(2次元地図からの2次元地図を使用)
- このとき地図生成も行う
    ```python
    $ roslaunch sobit_navigation sobit_pro_navigation_gmapping.launch
    ``` -->

---

[Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)

