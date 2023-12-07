<!-- # [SOBIT Mapping](/sobit_mapping)  
- 地図生成パッケージ
- 目次
    1. [2次元地図生成(gmapping)](/doc/readme/sobit_mapping_gmapping.md)
    2. [3次元地図生成(octomap)](/doc/readme/sobit_mapping_octomap.md)
    3. [地点登録](/doc/readme/sobit_mapping_create_location_file.md)

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack) -->

# 自律地図生成
オープンソース[explore_lite](http://wiki.ros.org/explore_lite)を使って，ロボットが自律的に地図生成を行う． 
1. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
2. 自律地図生成を起動する 
    - SOBIT PROで自律地図生成[sobit_pro_active_slam.launch](/sobit_mapping/launch/sobit_pro_active_slam.launch)を起動 \
        ```sh
        $ roslaunch sobit_mapping sobit_pro_active_slam.launch
        ```
    - SOBIT EDU，SOBIT MINIで自律地図生成[sobit_turtlebot_active_slam.launch](/sobit_mapping/launch/sobit_turtlebot_active_slam.launch)を起動 
        ```sh
        $ roslaunch sobit_mapping sobit_turtlebot_active_slam.launch
        ```
3. Navigationしたい環境の地図を保存 \
    Rvizの地図を見て，ほしい地図ができていたらsavecomaandのターミナルでEnterボタンを押して地図を保存する．\
    保存された地図画像と，その詳細情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時のファイル名で保存される．


# カメラを用いた地図生成
1. カメラの点群のトピック名を揃える \
    [gmapping_multi_sensor.launch](/sobit_mapping/launch/gmapping_multi_sensor.launch)の6行目のremapをtoを点群のトピック名にする．\
    ロボットに搭載されているカメラによって，点群のトピック名が異なるので，詳しくは各ロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))で点群名をチェック．\
    例えば，azure_kinectは"/points2"，realsenseは"/head_camera/depth_registered/points"である． 
2. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
3. カメラの点群を2D LiDARと統合した地図生成を起動 \
    以下のコマンドで起動．
    ```sh
    $ roslaunch sobit_mapping gmapping_multi_sensor.launch
    ```
4. 人間が操作できるように[teleop.launch](/sobit_mapping/launch/teleop.launch)を起動 \
    以下のコマンドで起動する． 
    ```sh
    $ roslaunch sobit_mapping teleop.launch
    ```
5. ロボットを操作して，Navigationしたい環境の地図を保存 \
    起動したターミナルで操作方法を確認しながら，Rvizの地図を見てロボットを操作する．\
    地図ができたら，save_map_comaandのターミナル(青いターミナル)でEnterボタンを押して地図を保存する．\
    保存された地図画像(pgmファイル)と，その詳細情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時のファイル名で保存される．

> [!NOTE]
> ロボットを起動させた状態では，カメラが正面向いているので，ロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を参照して，カメラを下に向けたりしても良い．


# ロボットを用いずに地点登録
1. 生成した地図のパスを登録する \
    [create_location_file.launch](/sobit_mapping/launch/create_location_file.launch)のuse_robotをfalseにし，さらにmap_fileを書き換える． \
    map_fileは，自分で生成した地図を指定する．\
    例えば，[example.pgm](/sobit_mapping/map/example.pgm)というマップの場合は，以下のように指定する．
    ```xml
    <arg name="use_robot" default="false"/>
    <arg name="map_file" default="$(find sobit_mapping)/map/example.yaml"/>
    ```
    ※ 拡張子が.ymalになることに注意．直接画像ファイルを指定するのではなく，地図のymalデータファイルを指定する．
2. 地点登録を起動する \
    以下のコマンドで起動する． \
        ```sh
        $ roslaunch sobit_mapping create_location_file.launch
        ```
3. 地点を登録する \
    起動したRvizで，2D Nav Goalをmapにクリックする．
    そこで地点登録のターミナルに地点名を入力し，Enterを押してその位置を登録．
4. 保存 \
    この手順で全ての地点を登録する．\
    地点登録が終わったら，端末で「q」と入力し，最後に適当に2D Nav Goalをクリックして終了する．\
    地点登録された情報が入ったymalデータは，[map](/sobit_mapping/map/)に，"map_location_"+"保存した日時"のファイル名で保存される．


# 地点登録確認・追加
1. 生成した地図，参照・追加した地点登録ファイルのそれぞれのパスを登録する \
    [location_file_viewer.launch](/sobit_mapping/launch/location_file_viewer.launch)のmap_file，location_file_pathをそれぞれ書き換える． \
    map_fileは，自分で生成した地図を，location_file_pathは地点登録ファイルを指定する．\
    例えば，[example.pgm](/sobit_mapping/map/example.pgm)というマップで，地点登録ファイルが[map_location_example.ymal](/sobit_mapping/map/map_location_example.ymal)という地点登録ファイルの場合は，以下のように指定する．
    ```xml
    <arg name="map_file" default="$(find sobit_mapping)/map/example.yaml"/>
    <arg name="location_file_path" default="$(find sobit_mapping)/map/map_location_example.yaml"/>
    ```
2. 地点登録・追加のノードを起動する \
    [location_file_viewer.launch](/sobit_mapping/launch/location_file_viewer.launch)を起動する．
    ```sh
    $ roslaunch sobit_mapping location_file_viewer.launch
    ```
3. 確認・追加 \
    起動したRvizをみて，赤い矢印が既に登録されている地点として確認する． \
    追加する場合は，Rvizの2D Nav Goalでクリックし，ターミナルで新たな地点名を登録する． \
    Rviz上に追加された地点が青く表示されたら追加完了． \
4. 保存 \
    確認・追加が終わったら，端末で「q」と入力し，最後に適当に2D Nav Goalをクリックして終了する．