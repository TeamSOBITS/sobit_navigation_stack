
# 3D カメラによるNavigationの詳細設定

## noise_colorレイヤーのセットアップ
1. 床の色の登録のための設定\
    [sobit_mapping/launch/flor_color_setup.launch](/sobit_mapping/launch/flor_color_setup.launch)のcloud_topic_nameを使用する3D カメラの点群(sensor_msgs/PointCloud2)が格納されているものを指定する．
    例えばazure kinectならば，"/points2"である．
    以下は，realsenseカメラを用いる場合の例
    ```xml
    <arg name="cloud_topic_name"              default="/head_camera/depth_registered/points"/>  <!-- azure_kinect="/points2", realsense="/head_camera/depth_registered/points" -->
    ```
    また，床を見るためにロボットのカメラを下に向けることができる．\
    use_sobit_に，SOBIT PROを使う場合はpro，SOBIT EDUを使う場合はedu，SOBIT MINIを使う場合はminiと指定する．
    そのとき，pan_angle_deg，tilt_angle_degはカメラをどれだけ回転させるかをパン・チルト回転で表記する．（単位はdegree）\
    以下は，SOBIT PROを用いて，カメラを左右に0度，下に40度変えてNavigationする場合の例
    ```xml
    <arg name="use_sobit_"                    default="pro"    />   <!-- pro or edu or mini -->
    <arg name="pan_angle_deg"                 default="0.0"    />
    <arg name="tilt_angle_deg"                default="-40.0"  />
    ```

2. 床の色を登録
    ロボット本体と，3D カメラを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．\
    ```sh
    $ roslaunch sobit_mapping flor_color_setup.launch
    ```

3. 床の色を決定する\
    床の色が画面上に表示される．\
    床の色として適切ならばターミナルの指示に従って1を，適切でない場合は2を押してやり直すことができる．\
    これは，1を選択するまで，適切な色になるよう何度でもやり直すことができる．\
    床の色として正しくなれば1を押してその色のRGB値を[sobit_navigation/param/floor_color/rgb_base.yaml](/sobit_navigation/param/floor_color/rgb_base.yaml)に保存される．\
    ここを参照してこの値から外れていればコストとしている．


## objectsレイヤーのセットアップ