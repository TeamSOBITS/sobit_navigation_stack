# SOBIT Navigation Stack
SOBIT EDU, MININ, PROのための自律移動パッケージ  

## Before Use
```bash
$ bash ~/catkin_ws/src/sobit_navigation_stack/install.sh 
```

# Manual
- [SOBIT Mappingの使い方](sobit_mapping)
    1. [2次元地図生成(gmapping)](doc/readme/sobit_mapping_gmapping.md)
    2. 3次元地図生成
        - [RTABMap](doc/readme/sobit_mapping_rtabmap.md)
        - [Octomap](doc/readme/sobit_mapping_octomap.md)
    3. [地点登録](doc/readme/sobit_mapping_create_location_file.md)
- [SOBIT Navigationの使い方](sobit_navigation)
- [SOBIT Navigation Libraryの使い方](sobit_navigation_library)

# Tips
- [AzureKinectでRTABMapを実行するための注意点](doc/readme/azure_kinect_rtabmap.md)
- [障害物回避のためのパラメータ調整](doc/readme/obstacle_avoidance.md)
- [全方向移動するためのパラメータ設定](doc/readme/omni_directional_mobile.md)
- [projected_map(立体的な障害物を押しつぶした2次元地図)を用いたナビゲーションをする方法](doc/readme/projected_map_navigation.md)
- [dwa_local_plannerのパラメータについて](doc/readme/dwa_params.md)
- [amclのパラメータについて](doc/readme/amcl_params.md)
- [recovery_behaviorsについて](doc/readme/recovery_behaviors.md)
- [rtabmapについて](doc/readme/rtabmap.md)

# [ROS Navigation Stack ソフトウェア設計仕様](https://robo-marc.github.io/navigation_documents/)
- 作成：産業技術総合研究所・ロボットイノベーションセンター

- [はじめに](https://robo-marc.github.io/navigation_documents/introduction.html)
- [Navigation Stack概要](https://robo-marc.github.io/navigation_documents/navigation_overview.html#)
    - [Navigation Stackとは](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id1)
    - [Navigation Stackの入出力](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id2)
        - [Transform Tree](https://robo-marc.github.io/navigation_documents/navigation_overview.html#transform-tree)
        - [測域センサ情報（レーザースキャン）](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id4)
        - [測域センサ情報（ポイントクラウド）](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id6)
        - [オドメトリ情報](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id8)
        - [地図](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id10)
        - [駆動（速度）命令](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id12)
        - [その他のメッセージ型](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id14)
        - [サービス型](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id22)
        - [アクション型](https://robo-marc.github.io/navigation_documents/navigation_overview.html#id26)
- [各パッケージ仕様](https://robo-marc.github.io/navigation_documents/packages.html)
    - [move_baseメイン](https://robo-marc.github.io/navigation_documents/move_base.html)
        - [move_base](https://robo-marc.github.io/navigation_documents/move_base.html)
        - [nav_core](https://robo-marc.github.io/navigation_documents/nav_core.html)
    - [自己位置推定関連](https://robo-marc.github.io/navigation_documents/packages.html#id2)
        - [amcl](https://robo-marc.github.io/navigation_documents/amcl.html)
        - [fake_localization](https://robo-marc.github.io/navigation_documents/fake_localization.html)
    - [地図配信](https://robo-marc.github.io/navigation_documents/packages.html#id3)
        - [map_server](https://robo-marc.github.io/navigation_documents/map_server.html)
    - [コストマップ関連](https://robo-marc.github.io/navigation_documents/packages.html#id4)
        - [costmap_2d](https://robo-marc.github.io/navigation_documents/costmap_2d.html)
        - [voxel_grid](https://robo-marc.github.io/navigation_documents/voxel_grid.html)
    - [グローバルプランナー関連](https://robo-marc.github.io/navigation_documents/packages.html#id5)
        - [nav_fn](https://robo-marc.github.io/navigation_documents/navfn.html)
        - [global_planner](https://robo-marc.github.io/navigation_documents/global_planner.html)
        - [carrot_planner](https://robo-marc.github.io/navigation_documents/carrot_planner.html)
    - [ローカルプランナー関連](https://robo-marc.github.io/navigation_documents/packages.html#id6)
        - [base_local_planner](https://robo-marc.github.io/navigation_documents/base_local_planner.html)
        - [dwa_local_planner](https://robo-marc.github.io/navigation_documents/dwa_local_planner.html)
    - [リカバリー動作関連](https://robo-marc.github.io/navigation_documents/packages.html#id7)
        - [clear_costmap_recovery](https://robo-marc.github.io/navigation_documents/clear_costmap_recovery.html)
        - [rotate_recovery](https://robo-marc.github.io/navigation_documents/rotate_recovery.html)
        - [move_slow_and_clear](https://robo-marc.github.io/navigation_documents/move_slow_and_clear.html)

---

# [ROS Navigation Tuning Guide](https://kaiyuzheng.me/documents/navguide.pdf)
```
ROSナビゲーションスタックは、移動ロボットが場所から場所へ確実に移動するために威力を発揮します。ナビゲーションスタックの仕事は、オドメトリ、センサー、環境マップからのデータを処理して、ロボットが実行するための安全な経路を生成することです。このナビゲーションスタックの性能を最大限に引き出すには、パラメータの微調整が必要ですが、これは見た目ほど簡単なことではありません。しかし、この作業は見た目ほど簡単ではありません。概念や推論が未熟な人は、手当たり次第に試してしまい、多くの時間を浪費してしまいます。

この記事は、ナビゲーションパラメータの微調整のプロセスを通じて読者を導くことを意図しています。どのように」「なぜ」調整するのかを知るための参考資料です。主要なパラメータの値を設定する際に このガイドでは、読者が以下を完了していることを想定しています。すでにナビゲーションスタックをセットアップし、最適化する準備ができています。これはまた、以下の要約でもあります。ROSナビゲーションスタックに関する私の研究の成果です。
```
```
The ROS navigation stack is powerful for mobile robots to move from place to place reliably. The job of navigation stack is to produce a safe path for the robot to execute, by processing data from odometry, sensors and environment map. Maximizing the performance of this navigation stack requires some fine tuning of parameters, and this is not as simple as it looks. One who is sophomoric about the concepts and reasoning may try things randomly, and wastes a lot of time.

This article intends to guide the reader through the process of fine tuning navigation parameters. It is the reference when someone need to know the ”how” and ”why” when setting the value of key parameters. This guide assumes that the reader has already set up the navigation stack and ready to optimize it. This is also a summary of my work with the ROS navigation stack.
```

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)
