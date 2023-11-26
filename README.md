<a name="readme-top"></a>

[JP](template_readme.md) | [EN](template_readme_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# Sobit Navigation Stack

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
    <li><a href="#実行・操作方法">実行・操作方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#変更履歴">変更履歴</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

SOBIT PRO，SOBIT EDU，SOBIT MINIのための自律移動パッケージ．\
Navigationのオープンソースの概要は[こちら](https://robo-marc.github.io/navigation_documents/introduction.html)をチェック．\
また自律移動の仕組みについても，[ROSのオープンソース](https://robo-marc.github.io/navigation_documents/navigation_overview.html#)を参照．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- セットアップ -->
## セットアップ
本レポジトリのセットアップ方法について説明します．

### 環境条件

必要な外部ソフトや正常動作を確認した環境について説明してください．
| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.0~ |

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ cd　~/catkin_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/sobit_navigation_stack.git
   ```
3. レポジトリの中へ移動します．
   ```sh
   $ cd sobit_navigation_stack
   ```
4. 依存パッケージをインストールします．
   1. 通常のインストール方法
        ```sh
        $ bash install.sh
        ```
   2. Navigationを深く勉強したい人向け
        ```sh
        $ bash install.sh clone_mode
        ```
        Navigation関係のパッケージをgit cloneによりインストールすることで，プログラムコードを編集して改良することができる \
        プログラムコードは[パッケージ一覧](/sobit_navigation_packages/)のところにまとめてインストールされる
5. パッケージをコンパイルします．
   ```sh
   $ cd ~/catkin_ws/
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- 実行・操作方法 -->
## Navigationの主な使い方

Navigationを使う上での基本的な流れ
1. 地図生成
    - 目的地まで，障害物を回避した経路を生成するため，ロボットが事前に地図を知る必要がある
    - 地図の障害物のデータと，現在ロボットが取得しているデータから，ロボットが現在どこにいるのかを推測する
2. 地点登録
    - 生成した地図の，どの位置からどの位置までの経路を生成するかのポイントとなる位置を登録する
3. actionlibによって呼び出す
    - ロボットの現在の地点から登録した地点まで，地図上の障害物がない安全なエリアに地図生成をする
    - 到着まで時間がかかることから，結果だけでなく途中経過も発信することのできるactionlib通信を用いる

### 地図生成
- ロボットを人間が操作して地図生成
1. ロボットを起動する
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
2. 地図生成を起動
    - SOBIT PROで地図生成
        ```sh
        $ roslaunch sobit_mapping sobit_pro_gmapping.launch
        ```
    - SOBIT EDU，SOBIT MINIで地図生成
        ```sh
        $ roslaunch sobit_mapping sobit_turtlebot_gmapping.launch
        ```
3. 人間が操作できるように[teleop.launch](/sobit_mapping/launch/teleop.launch)を起動
    ```sh
    $ roslaunch sobit_mapping teleop.launch
    ```
4. ロボットを操作して，Navigationしたい環境の地図を保存
    起動したターミナルで操作方法を確認しながら，Rvizの地図を見てロボットを操作する．\
    地図ができたら，savecomaandのターミナルでEnterボタンを押して地図を保存する．\
    保存された地図画像と，その詳細情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時のファイル名で保存される．
- 自律地図生成
オープンソース[explore_lite](http://wiki.ros.org/explore_lite)を使って，ロボットが自律的に地図生成を行う．
1. ロボットを起動する
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
2. 自律地図生成を起動する
    - SOBIT PROで自律地図生成[sobit_pro_active_slam.launch](/sobit_mapping/launch/sobit_pro/sobit_pro_active_slam.launch)を起動
        ```sh
        $ roslaunch sobit_mapping sobit_pro_active_slam.launch
        ```
    - SOBIT EDU，SOBIT MINIで自律地図生成[sobit_turtlebot_active_slam.launch](/sobit_mapping/launch/sobit_turtlebot/sobit_turtlebot_active_slam.launch)を起動
        ```sh
        $ roslaunch sobit_mapping sobit_turtlebot_active_slam.launch
        ```
3. Navigationしたい環境の地図を保存
    Rvizの地図を見て，ほしい地図ができていたらsavecomaandのターミナルでEnterボタンを押して地図を保存する．\
    保存された地図画像と，その詳細情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時のファイル名で保存される．
### 地点登録
- ロボットを使って地点登録
1. 生成した地図のパスをNavigationに登録する
    - SOBIT PRO用
        [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)のmap_fileを書き換える
    - SOBIT EDU，SOBIT MINI用
        [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)のmap_fileを書き換える
    [example.pgm](/sobit_mapping/map/example.pgm)という，事前にこちらで用意したマップを例に以下に示す．
    ```xml
        <arg name="map_file" default="$(find sobit_mapping)/map/example.yaml"/>  <!-- 拡張子に注意(.ymal) -->
    ```
2. ロボットを起動する
3. 地点登録を起動する
    - SOBIT PRO用
        ```sh
        $ roslaunch sobit_mapping sobit_pro_create_location_file.launch
        ```
    - SOBIT EDU，SOBIT MINI用
        ```sh
        $ roslaunch sobit_mapping sobit_turtlebot_create_location_file.launch
        ```
4. 登録したい地点にロボットを移動する
    - [teleop.launch](/sobit_mapping/launch/teleop.launch)を用いてロボットを移動 
        ```sh
        $ roslaunch sobit_mapping teleop.launch
        ```
    - Rvizの2D Nav Goalを用いてロボットを移動
5. 移動後，ロボットの現在地点を登録する
    - 3で起動した地点登録の端末に，地点名を入力
    - Enterを押して登録完了
    - 全ての登録したい地点を登録するまで4と5を繰り返す
6. Rvizに表示された地点登録情報(矢印)を保存
    - 地点登録が終わったら，端末で「q」と入力して保存
    - 地点登録された情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時+"_location"のファイル名で保存される．
- ロボットを使わずに地点登録
#### SOBIT EDU，SOBIT MINIで地図生成
1. ロボットを起動する．
<!-- デモの実行方法やスクリーンショットがあるとわかりやすくなるでしょう -->
1. [human_feature_detect.launch](/launch/human_feature_detect.launch)というlaunchファイルを実行します．
    ```sh
    $ roslaunch human_feature_detect human_feature_detect.launch
    ```
    これによって，画像から推論を行えるROSのService通信のServerが起動します．
2. [任意] imagesフォルダにあるサンプル画像([sample_image.jpg](/images/sample_image.jpg))を使って推論をしてみましょう．
    exampleコードを準備したので，それを使っていきます．
    Pythonの場合
   1. Pythonの場合
        ```sh
        $ rosrun human_feature_detect sample_2d.py
        ```
   2. C++の場合
        ```sh
        $ rosrun human_feature_detect sample_2d
        ```
    人の顔にバウンディングボックスがあてられ，性別と年齢を推定した結果の画像が出力されました．
    出力された画像は，[sample_image_result.jpg](/images/sample_image_result.jpg)として保存されています．

### 3次元で行える特徴検出（身長と服の色）
1. 点群のTopic名を設定します．paramとして[human_feature_detect.launch](/launch/human_feature_detect.launch)ファイルのに設定します．
   ```xml
    <param name="topic_name" value="/points2"/>
    ...
   ```
   他のパラメータについて
   ```
    <param name="target_frame" value="base_footprint"/>　　<!-- ロボットの基準フレーム。これによって身長を地面を基準とする頭の高さとできる -->
    <param name="face_range" value="0.20"/>               <!-- 顔の大体の大きさ。服の色を測る際に頭の先からどれだけ下の点群を参照するか -->
    <param name="clothes_range" value="0.35"/>            <!-- 服のおおよその縦幅。服の色を測る際、どれだけ広範囲を参照するか -->
   ```
2. 設定が完了したら，[human_feature_detect.launch](/launch/human_feature_detect.launch)というlaunchファイルを実行します．
    ```sh
    $ roslaunch human_feature_detect human_feature_detect.launch
    ```
    これによって，点群から推論を行えるROSのService通信のServerが起動します．
3. [任意] imagesフォルダにあるサンプル画像([sample_image.jpg](/images/sample_image.jpg))を使って推論をしてみましょう．
    exampleコードを準備したので，それを使っていきます．
    Pythonの場合
   1. Pythonの場合
        ```sh
        $ rosrun human_feature_detect sample_3d.py
        ```
   2. C++の場合
        ```sh
        $ rosrun human_feature_detect sample_3d
        ```
    ターミナルに，身長と服の色が出力されました．

> [!NOTE]
> このexampleコードを使えば，ロボットのカメラから得た画像や点群から，データをServiceのServerに送信することで，人の特徴を推定することができます．\
> [example](/example/)フォルダを確認し，それぞれのサンプルファイルからServiceのクライアント(リクエスト側)について学びましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- マイルストーン -->
<!-- ## マイルストーン

- [x] 目標 1
- [ ] 目標 2
- [ ] 目標 3
    - [ ] サブ目標

現時点のバッグや新規機能の依頼を確認するために[Issueページ](https://github.com/github_username/repo_name/issues) をご覧ください．

<p align="right">(<a href="#readme-top">上に</a>)</p> -->



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

* []()

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
<!-- [license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt -->




















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
