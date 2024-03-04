# Navigationをいろいろ工夫する
Navigationでは，障害物の検出方法ごとに，レイヤーと呼ばれる層を設けている．\
通常2D LiDARにより検出された障害物は"scan"という層で，障害物のコストとして扱われ，回避を行うことができる． \
しかし，椅子やテーブルのような立体的な障害物や，反対にLiDARよりも低い位置にある障害物検出できない．
そこで3D cameraを使うことで，対応できるようにする． \
2D LiDARで取得できなかった障害物を元に，カメラから得た別のレイヤーとして設けることで，その障害物を避けることができる． \
ここでは，いろいろな障害物の検出方法から，いくつかの層について説明をする． \
また，そのレイヤーの使い方についても説明する．


## scanレイヤー
2D LiDARによって得られた比較的広範囲の障害物をコストとして捉えることができるレイヤー．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| 水平方向に広範囲 | 固定の高さしか検出できない |
| 処理が早い | 薄い情報量 |

## bumperレイヤー
turtlebotの前方にあるバンパーが押されたときに停止するシステム．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| LiDARよりも低い物体を検知 | turtlebotにしかない |
|  | 衝突しないと検出しない |

## obstacleレイヤー
3D カメラから得られる全ての点群データから，床面を除去した凸部分をコストとする．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| LiDARよりも低い物体を検知 | 使用するカメラの点群の精度に依存する |
| 立体的な物体を検出できる | 床とほぼ同じ高さの物体は床と判定される |

## noise_colorレイヤー
全ての点群データから，床面と近い色を除去して，色が違う部分をコストとする．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| 物体の大きさに依存しない | 使用するカメラの点群の精度に依存する |
| 床と色が違えば高さによらず検出できる | 床の色が単色のときにしか使えない |

## objectsレイヤー
画像から物体を検出し，その位置の点群をコストとする．
| メリット  | デメリット |
| -------------------------- | -------------------------- |
| 物体の大きさに依存しない | 画像のピクセル数や点群の処理速度に依存 |
| 床の柄が複雑でも適切な検出が可能 | 学習している物体しか検出できない |




-----------------------------以下修正前のもの-----------------------------



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

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)

