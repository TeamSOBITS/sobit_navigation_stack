<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS Navigation Stack

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
      <ul>
        <li><a href="#地図生成">地図生成</a></li>
        <li><a href="#地点登録">地点登録</a></li>
        <li><a href="#ナビゲーションを実行">ナビゲーションを実行</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

SOBIT PRO，SOBIT EDU，SOBIT MINI，HSR(Simulation)のための自律移動パッケージ．\
Navigationのオープンソースの概要は[こちら](https://docs.nav2.org/)をチェック．\
また自律移動の仕組みについても，[ROSのオープンソース](https://github.com/ros-navigation/navigation2)を参照．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## 環境構築
本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |
| Python | 3.0~ |
| Simulator(使用する場合) | Sigverse ROS |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法

1. ROS2の`src`フォルダに移動します．
   ```sh
   cd　~/colcon_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   git clone -b humble-devel https://github.com/TeamSOBITS/sobits_navigation_stack.git
   ```
3. レポジトリの中へ移動します．
   ```sh
   cd sobits_navigation_stack
   ```
4. 依存パッケージをインストールします．
    ```sh
    bash install.sh
    ```

5. パッケージをコンパイルします．
   ```sh
   cd ~/colcon_ws/
   colcon build --symlink-install
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- 実行・操作方法 -->
## 実行・操作方法

Navigationを使う上での基本的な流れ
1. 地図生成 
    - 目的地まで障害物を回避した経路を生成するため，ロボットが事前に地図を知る必要がある
    - 地図の障害物のデータと，現在ロボットが取得しているデータから，ロボットが現在どこにいるのかを推測する
2. 地点登録 
    - 生成した地図の，どの位置からどの位置までの経路を生成するかのポイントとなる位置を登録する
3. Action通信で呼び出す
    - ロボットの現在の地点から登録した地点まで，地図上の障害物がない安全なエリアに経路生成をする
    - 到着まで時間がかかることから，結果だけでなく途中経過も発信することのできるAction通信を用いる

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 地図生成
1. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．\
    HSR(シミュレータ)の場合はsigverseやHSR本体のセンサデータを使えるように起動する．
2. 地図生成を起動\
    [gmapping.launch.py](/sobits_mapping/launch/gmapping.launch.py)
    にある
    **robot_name**を使用するロボットに切り換える．

    その後以下のコマンドで起動． 
    起動後に地図を保存するか聞かれるが，一旦無視する．
    ```sh
    ros2 launch sobits_mapping gmapping.launch.py
    ```

3. 人間が操作できるように[teleop.launch.py](/sobits_mapping/launch/teleop.launch.py)を起動\
    [teleop.launch.py](/sobits_mapping/launch/teleop.launch.py)にある**velocity_topic_name**を使用するロボットのトピック名に切り換える．

    その後以下のコマンドで起動．
    ```sh
    ros2 launch sobits_mapping teleop.launch.py
    ```

4. ロボットを操作して，Navigationしたい環境の地図を保存 \
    起動したxtermターミナル(青いターミナル)で操作方法を確認しながら，Rvizの地図を見てロボットを操作する．\
    **地図生成が完了したら，地図を保存する.**
5. 新たに地図ファイルを作成した場合はcolcon buildを実行する．\
   既存の地図ファイルと置き換えて作成した場合はcolcon buildを実行する必要はない．
    ```sh
    cd　~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 地点登録
1. 生成した地図のパスを指定する．
    [create_location_file.launch.py](/sobits_mapping/launch/create_location_file.launch.py)のmapを書き換える．\
    mapは，自分で生成した地図を指定する．\
    例えば，[map_example.pgm](/sobits_mapping/map/map_example.pgm)というマップの場合は，以下のように指定する．
    ```sh  
    DeclareLaunchArgument(
            # mapのファイルパス
            'map', default_value=os.path.join(get_package_share_directory("sobits_mapping"), 'map', 'map_example.yaml')
        ),
    ```
    ※ 拡張子が.ymalになることに注意．直接画像ファイルを指定するのではなく，地図のymalデータファイルを指定する．
2. 実機で地点登録するかどうかを設定する
    - 実機で地点登録**しない**場合\
         [create_location_file.launch.py](/sobits_mapping/launch/create_location_file.launch.py)の
         **use_robot**をfalseにする．
        ```sh
        'use_robot', default_value='false'
        ```
    - 実機で地点登録**する**場合\
        はじめに[create_location_file.launch.py](/sobits_mapping/launch/create_location_file.launch.py)の**use_robot**をtrueにし，**robot_name**を使用するロボットに変更する．
        ```sh
        'use_robot', default_value='true'
        ```
        <!-- 次に[nav2.launch.py](/sobits_navigation/launch/nav2.launch.py)
      の
      ```sh
      declare_velocity_topic_name_cmd = DeclareLaunchArgument(
        'velocity_topic_name',
        # default_value="/sobit_pro/cmd_vel",  ## SOBIT PRO ##
        # default_value="/sobit_edu/commands/velocity",  ## SOBIT EDU ##
        default_value="/sobit_mini/commands/velocity",  ## SOBIT MINI ##
        # default_value="/sobit_light/cmd_vel",  ## SOBIT LIGHT ##
        #default_value="/hsrb/command_velocity",  ## HSR(Simulation) ##
        description='velocity topic name')
      ```
      を，使用するロボットのトピック名に変更する． -->

3. 実機で地点登録する場合はロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる． \
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．\
    HSR(シミュレータ)の場合はsigverseやHSR本体のセンサデータを使えるように起動する．
4. 地点登録を起動する \
    以下のコマンドで起動する．
    ```sh
    ros2 launch sobits_mapping create_location_file.launch.py
    ```
    起動後，**地点登録を始める前に地点登録ファイルを保存する．**

5. 地点を登録する
    - 実機で地点登録**しない**場合\
      RVIZ上で，**2D Goal Pose**を選択し，登録したい位置と向きでmapにクリック
  
    - 実機で地点登録**する**場合\
    ロボットを地点登録したい位置まで移動させて地点登録する．\
    ロボットの移動のさせ方は以下2通りがある．
      - Navigationの機能を用いる場合\
        RVIZ上で，**2D Goal Pose**を選択し，mapにクリックする． 
      - 地図生成したときのように人間が操作する場合\
        以下のコマンドで実行
        ```sh
        ros2 launch sobits_mapping teleop.launch.py
        ```
    - ADD LOCATION：地点名を入力して登録
    - Delete　　　：登録した地点を削除
    - Rename　　　：登録した地点名を変更
7. すべての地点登録が終了したら，起動しているlaunchをすべて終了させる．\
   新たに地点登録ファイルを作成した場合はcolcon buildを実行する．
    ```sh
    cd　~/colcon_ws/
    colcon build --symlink-install
    ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### ナビゲーションを実行
1. mapを地図生成した地図に書き換える\
    Navigationに地図を登録する．\
    [nav2.launch.py](/sobits_navigation/launch/nav2.launch.py)のmapを作成した地図のファイル名に書き換える．\
    \
    例：作成した地図のファイル名がmap_example.yamlのとき

    ```sh
    default_value=os.path.join(get_package_share_directory('sobits_mapping'), 'map', 'map_example.yaml'),
    ```

    ここで書くのは，地図データです．地点登録のファイルと間違わないようにしてください．

2. 地点登録した情報を登録する \
    [nav2.launch.py](/sobits_navigation/launch/nav2.launch.py)のlocation_file_pathを作成した地点登録ファイルに書き換える．\
    \
    例：作成した地点登録ファイル名がlocation_example.yamlのとき
    ```sh
    default_value=os.path.join(
            get_package_share_directory('sobits_mapping'), 'location', 'location_example.yaml'),
    ```
3. [nav2.launch.py](/sobits_navigation/launch/nav2.launch.py)の
**robot_name**を使用するロボット名に書き換える．

4. ロボットを起動する \
    ロボット本体と，2D-LiDARを起動させる．\
    詳しくは，それぞれのロボットのgit hub([PRO](https://github.com/TeamSOBITS/sobit_pro.git)，[EDU](https://github.com/TeamSOBITS/sobit_edu.git)，[MINI](https://github.com/TeamSOBITS/sobit_mini.git))を確認．\
    HSR(シミュレータ)の場合はsigverseやHSR本体のセンサデータを使えるように起動する．
5. Navigationを起動する \
    以下のコマンドでNavigationを起動する． 
    ```sh
    ros2 launch sobits_navigation nav2.launch.py
    ```
    これによりマップとその上に地点登録したTFが出ていると思います．

5. アクションクライアントを起動する \
    これは基本的にプログラムから起動する．\
    地点登録した地点名ならどこにでも移動することが可能．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- マイルストーン -->
## マイルストーン

- [ ] 自律地図生成のREADMEを書く
- [ ] カメラを用いた地図生成
- [ ] 障害物のレイヤーのカスタム
    - [ ] bumperレイヤー
    - [ ] noise_colorレイヤー
    - [ ] objectsレイヤー

現時点のバッグや新規機能の依頼を確認するために[Issueページ](issues-url) をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


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
