<!-- # [SOBIT Mapping](/sobit_mapping)  
- 地図生成パッケージ
- 目次
    1. [2次元地図生成(gmapping)](/doc/readme/sobit_mapping_gmapping.md)
    2. [3次元地図生成(octomap)](/doc/readme/sobit_mapping_octomap.md)
    3. [地点登録](/doc/readme/sobit_mapping_create_location_file.md)

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack) -->

# 自律地図生成
オープンソース[explore_lite](http://wiki.ros.org/explore_lite)を使って，ロボットが自律的に地図生成を行う． \
1. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
2. 自律地図生成を起動する \
    - SOBIT PROで自律地図生成[sobit_pro_active_slam.launch](/sobit_mapping/launch/sobit_pro_active_slam.launch)を起動 \
        ```sh
        $ roslaunch sobit_mapping sobit_pro_active_slam.launch
        ```
    - SOBIT EDU，SOBIT MINIで自律地図生成[sobit_turtlebot_active_slam.launch](/sobit_mapping/launch/sobit_turtlebot_active_slam.launch)を起動 \
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
    例えば，azure_kinectは"/points2"，realsenseは"/head_camera/depth_registered/points"である． \
2. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
3. カメラの点群を2D LiDARと統合した地図生成を起動 \
    以下のコマンドで起動．
    ```sh
    $ roslaunch sobit_mapping gmapping_multi_sensor.launch
    ```
4. 人間が操作できるように[teleop.launch](/sobit_mapping/launch/teleop.launch)を起動 \
    以下のコマンドで起動する． \
    ```sh
    $ roslaunch sobit_mapping teleop.launch
    ```
5. ロボットを操作して，Navigationしたい環境の地図を保存 \
    起動したターミナルで操作方法を確認しながら，Rvizの地図を見てロボットを操作する．\
    地図ができたら，save_map_comaandのターミナル(青いターミナル)でEnterボタンを押して地図を保存する．\
    保存された地図画像(pgmファイル)と，その詳細情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時のファイル名で保存される．

> [!NOTE]
> ロボットを起動させた状態では，カメラが正面向いているので，ロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を参照して，カメラを下に向けたりしても良い．