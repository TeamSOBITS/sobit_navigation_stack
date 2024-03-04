<a name="readme-top"></a>

[JP](README.md) | [EN](README_EN.md)

<!-- [![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url] -->
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
    <li><a href="#Navigationの主な使い方">Navigationの主な使い方</a></li>
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
    - 通常のインストール方法
        ```sh
        $ bash install.sh
        ```
    - Navigationを深く勉強したい人向け
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
    - ロボットの現在の地点から登録した地点まで，地図上の障害物がない安全なエリアに経路生成をする
    - 到着まで時間がかかることから，結果だけでなく途中経過も発信することのできるactionlib通信を用いる



### 地図生成
1. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
2. 地図生成を起動 \
    以下のコマンドで起動． 
    ```sh
    $ roslaunch sobit_mapping gmapping.launch
    ```
3. 人間が操作できるように[teleop.launch](/sobit_mapping/launch/teleop.launch)を起動 \
    以下のコマンドで起動する．
    ```sh
    $ roslaunch sobit_mapping teleop.launch
    ```
4. ロボットを操作して，Navigationしたい環境の地図を保存 \
    起動したターミナルで操作方法を確認しながら，Rvizの地図を見てロボットを操作する．\
    地図ができたら，save_map_command.pyのターミナル(青いターミナル)でEnterボタンを押して地図を保存する．\
    保存された地図画像(pgmファイル)と，その詳細情報が入ったymalデータは，[map](/sobit_mapping/map/)に保存した日時のファイル名で保存される．

> [!NOTE]
> これを人間がロボットを操作せずに，自律的に行えないのか？と思ったら[自律地図生成の仕方](/sobit_mapping/README.md/#自律地図生成)をチェック．

> [!NOTE]
> これは2D LiDARのみでの地図生成であるが，一脚のテーブルのような立体的な障害物は2D LiDARでの検出ができない．
> そこでロボットの3D cameraを用いることで解決する[カメラを用いた地図生成について](/sobit_mapping/README.md/#カメラを用いた地図生成)をチェック．


### 地点登録
1. 生成した地図のパスをNavigationに書き換える
    - SOBIT PROで地点登録 \
        [/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)のmap_fileを書き換える．
    - SOBIT EDU，SOBIT MINIで地点登録 \
        [/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)のmap_fileを書き換える．
    map_fileは，自分で生成した地図を指定する．\
    例えば，[example.pgm](/sobit_mapping/map/example.pgm)というマップの場合は，以下のように指定する．
    ```xml
    <arg name="map_file" default="$(find sobit_mapping)/map/example.yaml"/>
    ```
    ※ 拡張子が.ymalになることに注意．直接画像ファイルを指定するのではなく，地図のymalデータファイルを指定する． \ 
    また，[/sobit_mapping/launch/create_location_file.launch](/sobit_mapping/launch/create_location_file.launch)のuse_robotをtrueにする． \
2. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる． \ 
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
3. Navigationを起動する \
    ロボットによって，以下どちらかのコマンドでNavigationを起動する．
    - SOBIT PROで地点登録
    ```sh
    $ roslaunch sobit_navigation sobit_pro_navigation.launch
    ```
    - SOBIT EDU，SOBIT MINIで地点登録
    ```sh
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch
    ```
4. 地点登録を起動する \
    以下のコマンドで起動する．
    ```sh
    $ roslaunch sobit_mapping create_location_file.launch
    ```
5. 地点を登録する \
    ロボットのいる位置が登録される．\
    ロボットの移動のさせ方は以下2通りがあるので好きな方を選ぶ．
    - Navigationの機能を用いる \
    起動したRvizの2D Nav Goalをmapにクリックすることでロボットが移動する． 
    - Mappingしたときのように人間が操作 \
    [teleop.launch](/sobit_mapping/launch/teleop.launch)を用いてロボットを移動． \
    以下のコマンドで起動する． 
    ```sh
    $ roslaunch sobit_mapping teleop.launch
    ```
    ロボットを登録させたい位置まで移動． \
    そこで地点登録のターミナルに地点名を入力し，Enterを押して登録完了．
6. 保存 \
    この手順で地点登録したい全ての地点を登録する．\
    地点登録が終わったら，端末で「q」と入力して保存する．\
    地点登録された情報が入ったymalデータは，[map](/sobit_mapping/map/)に，"map_location_"+"保存した日時"のファイル名で保存される．

> [!NOTE]
> ここではロボットを使っての地点登録方法を書いたが，RoboCupでは競技環境を使える時間に限りがある．
> そのため，競技環境が使えるときは，なるべく競技練習の時間に充てることが望ましい．
> そこで，地点登録をロボットを使わずにすることで時間を有効にすることができる．
> 詳しい使い方は[ロボットを用いずに地点登録](/sobit_mapping/README.md/#ロボットを用いずに地点登録)をチェック．

> [!NOTE]
> 地点登録結果を見たり，追加したい場合は[こちら](/sobit_mapping/README.md/#地点登録確認・追加)．


### actionlibによって呼び出す（実際にNavigationする）
1. map_fileを生成したマップに書き換える
    - SOBIT PROでナビゲーション
    [sobit_pro_navigation.launch](/sobit_navigation/launch/sobit_pro/sobit_pro_navigation.launch)のmap_fileを地点登録のときと同様に書き換える．\
    既に書き換えられている場合はそのままでOK．
    - SOBIT EDU，SOBIT MINIでナビゲーション
    [sobit_turtlebot_navigation.launch](/sobit_navigation/launch/sobit_turtlebot/sobit_turtlebot_navigation.launch)のmap_fileを地点登録のときと同様に書き換える．\
    既に書き換えられている場合はそのままでOK．
2. 地点登録した情報をrosparamに登録する \
    [load_location_file.launch](/sobit_mapping/launch/load_location_file.launch)のrosparamのfileを，自分で生成した地点登録ファイルに書き換える．\
    例えば，[map_location_example.ymal](/sobit_mapping/map/map_location_example.ymal)という地点登録ファイルの場合は，以下のように指定する．
    ```xml
    <rosparam command="load" file="$(find sobit_mapping)/map/map_location_example.yaml"/>
    ```
    書き換えたら，以下のコマンドでrosparamに登録する． 
    ```sh
    $ roslaunch sobit_mapping load_location_file.launch
    ```
3. ロボットを起動する \ 
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．
4. Navigationを起動する \
    以下のコマンドでNavigationを起動する． 
    - SOBIT PROでナビゲーション
    ```sh
    $ roslaunch sobit_navigation sobit_pro_navigation.launch
    ```
    - SOBIT EDU，SOBIT MINIでナビゲーション
    ```sh
    $ roslaunch sobit_navigation sobit_turtlebot_navigation.launch
    ```
5. アクションクライアントを起動する \
    これは基本的にプログラムから起動する．\
    地点登録した地点名ならどこにでも移動することが可能．\
    移動する例として，[/sobit_navigation_library/example/move_location_example.py](/sobit_navigation_library/example/move_location_example.py)を起動．\
    このexampleコードでは，"table"という地点名の位置まで移動する． \
    以下のコマンドで実行．
    ```sh
    $ rosrun sobit_navigation_library move_location_example.py
    ```
> [!NOTE]
> この呼び出した[move_location_example.py](/sobit_navigation_library/example/move_location_example.py)や，C++での呼び出し方([move_location_example.cpp](/sobit_navigation_library/example/move_location_example.cpp))を使用したい場合，またいろいろなNavigationのツールについても，詳しくは[こちら](/sobit_navigation_library/README.md)をチェック．

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

* [ROS Navigationスタックソフトウェア設計仕様](https://robo-marc.github.io/navigation_documents/)
* [explore_lite](http://wiki.ros.org/explore_lite)

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
