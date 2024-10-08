<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# SOBIT Navigation Stack

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#地点登録">地点登録</a></li>
    <li><a href="#地点登録確認・追加">地点登録確認・追加</a></li>
    <li><a href="#Navigationの使い方">Navigationの使い方</a></li>
    <!-- <li><a href="#マイルストーン">マイルストーン</a></li> -->
    <!-- <li><a href="#変更履歴">変更履歴</a></li> -->
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

Kachaka(ROS2)のための自律移動パッケージ．

> [!NOTE]
> kachaka-apiがインストールされている必要があります.


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- セットアップ -->
## セットアップ
本レポジトリのセットアップ方法について説明します．

### 環境条件

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 22.04 (Focal Fossa) |
| ROS | Humble Hawksbill |
| Python | 3.10~ |

### インストール方法


1. ROSの`src`フォルダに移動します．
   ```sh
   $ cd　~/colcon_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone -b feature/kachaka https://github.com/TeamSOBITS/sobit_navigation_stack.git
   ```
3. パッケージをコンパイルします．
   ```sh
   $ cd ~/colcon_ws/
   $ colcon build
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- 実行・操作方法 -->



## 地点登録
1. Kachakaを起動し接続する \
2. アプリで地点登録したい地図を設定する\
3. 地点登録を起動する \
    ロボットを動かして地点登録する場合は[create_location_file_launch.py](/launch/create_location_file_launch.py)のuse_robotをtrueにする．\
    動かさない場合はfalseにする．\
    以下のコマンドで起動する．
    - 実機で地点登録
        ```sh
        $ ros2 launch sobit_navigation_stack create_location_file_launch.py
        ```
4. 地点を登録する\
    - use_robotをfalseにした場合\
        起動したRvizの2D Nav Goalをmapにクリックすることでロボットが移動する． 
    - use_robotをtrueにした場合\
        teleopでKachakaを登録させたい位置まで移動． \
    それぞれ地点登録のターミナルに地点名を入力し，Enterを押して登録完了．
5. 保存 \
    4を繰り返していくことで地点登録したい全ての地点を登録する．\
    地点登録が終わったら，端末で「q」と入力して保存する．\
    地点登録された情報が入ったymalデータは，[/location](/location/)に，「map_location_ + 保存した日時 + .yaml」のファイル名で保存される．


## 地点登録確認・追加
1. Kachakaを起動し接続する \
2. アプリで地点登録したい地図を設定する\
3. 地点登録のファイルパスを登録する \
    - [location_file_viewer_launch.py](/launch/location_file_viewer_launch.py)のlocation_fileをそれぞれ書き換える． \    
4. 地点登録・追加のノードを起動する \
    ```sh
    $ ros2 launch sobit_navigation_stack location_file_viewer_launch.py
    ```
5. 確認・追加 \
    起動したRvizをみて，赤い矢印が既に登録されている地点として確認する． \
    追加する場合は，Rvizの2D Nav Goalでクリックし，ターミナルで新たな地点名を登録する． \
    Rviz上に追加された地点が青く表示されたら追加完了．
6. 保存 \
    確認・追加が終わったら，端末で「q」と入力し，最後に適当に2D Nav Goalをクリックして終了する．



## Navigationの使い方

1. [navigation.py](/example/navigation.py)のグローバル変数と目的地を変更する
    - 地点登録したyamlファイルのパスに書き換える
    - 移動してほしい場所まで移動する
2. 以下のコマンドで起動する 
    - ```sh
        $ ros2 run sobit_navigation_stack navigation.py
        ```
3. actionlibによって呼び出す 
    - 登録された地点まで移動します


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- マイルストーン -->
## マイルストーン
<!-- 
- [x] 目標 1
- [ ] 目標 2
- [ ] 目標 3
    - [ ] サブ目標 -->

現時点のバッグや新規機能の依頼を確認するために[Issueページ](issues-url) をご覧ください．

<p align="right">(<a href="#readme-top">上に</a>)</p>



<!-- 変更履歴 -->
<!-- ## 変更履歴

- 2.0: 代表的なタイトル
  - 詳細 1
  - 詳細 2
  - 詳細 3
- 1.1: 代表的なタイトル
  - 詳細 1
  - 詳細 2
  - 詳細 3
- 1.0: 代表的なタイトル
  - 詳細 1
  - 詳細 2
  - 詳細 3 -->

<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- 参考文献 -->
## 参考文献

<!-- * [ROS Navigationスタックソフトウェア設計仕様](https://robo-marc.github.io/navigation_documents/)
* [explore_lite](http://wiki.ros.org/explore_lite) -->

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[license-url]: LICENSE




















<!-- # SOBIT Navigation Stack
SOBIT EDU, MININ, PROのための自律移動パッケージ  

## Before Use
```python
$ cd ~/catkin_ws/src
$ git clone https://github.com/TeamSOBITS/sobit_navigation_stack.git
$ cd sobit_navigation_stack
$ bash install.sh
```

# Manual
- [Tutorial](/doc/readme/tutorial.md)
- [SOBIT Mappingの使い方](sobit_mapping)
    1. [2次元地図生成(gmapping)](/doc/readme/sobit_mapping_gmapping.md)
    2. 3次元地図生成
        - [RTABMap](/doc/readme/sobit_mapping_rtabmap.md)
        - [Octomap](/doc/readme/sobit_mapping_octomap.md)
    3. [地点登録](/doc/readme/sobit_mapping_create_location_file.md)
- [SOBIT Navigationの使い方](sobit_navigation)
- [SOBIT Navigation Libraryの使い方](sobit_navigation_library)

# Tips
- [Tutorial](/doc/readme/tutorial.md)
- [AzureKinectでRTABMapを実行するための注意点](/doc/readme/azure_kinect_rtabmap.md)
- [障害物回避のためのパラメータ調整](/doc/readme/obstacle_avoidance.md)
- [projected_map(立体的な障害物を押しつぶした2次元地図)を用いたナビゲーションをする方法](/doc/readme/projected_map_navigation.md)
- [dwa_local_plannerのパラメータについて](/doc/readme/dwa_params.md)
- [amclのパラメータについて](/doc/readme/amcl_params.md)
- [recovery_behaviorsについて](/doc/readme/recovery_behaviors.md)
- [コストマップに付与されるコストを変える](/doc/readme/costmap_parameter_turning.md)

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
## Abstract
The ROS navigation stack is powerful for mobile robots to move from place to place reliably. The job of navigation stack is to produce a safe path for the robot to execute, by processing data from odometry, sensors and environment map. Maximizing the performance of this navigation stack requires some fine tuning of parameters, and this is not as simple as it looks. One who is sophomoric about the concepts and reasoning may try things randomly, and wastes a lot of time.

This article intends to guide the reader through the process of fine tuning navigation parameters. It is the reference when someone need to know the ”how” and ”why” when setting the value of key parameters. This guide assumes that the reader has already set up the navigation stack and ready to optimize it. This is also a summary of my work with the ROS navigation stack.

## 日本語アブストラクト
ROSナビゲーションスタックは、移動ロボットが場所から場所へ確実に移動するために威力を発揮します。ナビゲーションスタックの仕事は、オドメトリ、センサー、環境マップからのデータを処理して、ロボットが実行するための安全な経路を生成することです。このナビゲーションスタックの性能を最大限に引き出すには、パラメータの微調整が必要ですが、これは見た目ほど簡単なことではありません。しかし、この作業は見た目ほど簡単ではありません。概念や推論が未熟な人は、手当たり次第に試してしまい、多くの時間を浪費してしまいます。

この記事は、ナビゲーションパラメータの微調整のプロセスを通じて読者を導くことを意図しています。どのように」「なぜ」調整するのかを知るための参考資料です。主要なパラメータの値を設定する際に このガイドでは、読者が以下を完了していることを想定しています。すでにナビゲーションスタックをセットアップし、最適化する準備ができています。これはまた、以下の要約でもあります。ROSナビゲーションスタックに関する私の研究の成果です。 -->
