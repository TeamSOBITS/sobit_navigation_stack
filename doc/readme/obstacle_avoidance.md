# 障害物回避のためのパラメータ調整
静的障害物や動的障害物を避けるためのパラメータ調整について調査結果をまとめておきます

## 01．障害物回避のためのパラメータ
### コストマップのパラメータ
ナビゲーションを行う際、あらかじめ作成した環境全体の地図であるグローバルコストマップと
ロボットのセンサデータから作られる周辺環境の地図であるローカルコストマップを用いて環境認識を行います．

ここで，2次元地図を使用した場合，2次元センサ上で捉えられていない物体は地図に反映されない場合があります．
そのため，障害物衝突に危険性が考えられます．

そこで，3次元地図を用いて，より多くの障害物を捉えられるようにします．

詳しくはこちら
- [projected_map(立体的な障害物を押しつぶした2次元地図)を用いたナビゲーションをする方法](doc/readme/projected_map_navigation.md)

### DWA Plannerのパラメータ
ナビゲーションでは，グローバルコストマップと目標位置を使って経路生成をするグローバルパスプランと
ローカルコストマップと目標位置を使って，グローバルパスプランを沿う経路生成をするローカルパスプランの2つがあります．

グローバルパスプランでは，ダイクストラ法やA*アルゴリズム，ポテンシャル法などの経路生成アルゴリズムが使用されます([global_planner](http://wiki.ros.org/global_planner))

ローカルパスプランでは， Dynamic Window Approach (DWA)がが使用されます([base_local_planner](http://wiki.ros.org/base_local_planner?distro=noetic))

障害物回避で重要になるのが周辺環境から経路を生成するローカルパスプランになります．

詳しくはこちら
- [Dynamic Window Approach Tutorial(DWA)についてはこちら](https://gitlab.com/TeamSOBITS/path_planning_tutorial/-/tree/master/dwa_tutorial)
- [base_local_plannerのパラメータについてはこちら](https://gitlab.com/TeamSOBITS/sobit_navigation_stack/-/blob/main/doc/readme/dwa_params.md)

## 動的障害物の回避
- shutdown_costmaps   (bool, default false)
    - 障害物をコストマップに残し続けておくか。これをfalseにしておくと人等の移動物体はその移動軌跡が障害物となってコストマップに現れる。

## 02．リカバリーモード
ローカルパスプランにおいては，障害物の出現位置によっては，当初進めると思っていたのにそれ以上進めなくなってしまう状況が起こりえます．通せんぼ状態です．

そういう状態から，頑張って抜けだそうとするための仕組みが，[rotate_recovery](http://wiki.ros.org/rotate_recovery)です．

### rotate_recoveryの手順
1. できれば避けたいと思っていたコストマップ領域を少し狭めて，その場でローカルパスプランを行い，隙間を通れるところが無いか確認する
2. 1で失敗したら，一周回りながローカルパスプランを行い，隙間を通れるところが無いか確認する
3. 2で失敗したら，1. よりももっとコストマップ領域を狭めて，その場でローカルパスプランを行い，隙間を通れるところが無いか確認する
4. 3で失敗したら，2. と同様のローカルパスプランを行う
5. 4で失敗したら，潔く諦めます
上記フローのうち 1. ～4. のどこかで隙間を見つけられれば，再び元のナビゲーションに復帰します．

詳しくはこちら
- [move_base でRecovery 行動に遷移する条件を調べてみた](https://qiita.com/MoriKen/items/1f1f2d1e6ef0046ec12a)
- [積み状態からの回復](https://qiita.com/MoriKen/items/d5cd6208143d6c40caff#%E7%A9%8D%E3%81%BF%E7%8A%B6%E6%85%8B%E3%81%8B%E3%82%89%E3%81%AE%E5%9B%9E%E5%BE%A9)

## 03．最経路計画
ナビゲーション中にパスプランを再度行うためのパラメータについて説明します

- max_planning_retries   (int32_t, default -1)
    - base_local_plannerで有効なパスが見つからなかった時に何度までパスの計算をリトライできるか？リトライの後にrecovery behaviorsに入る。-1に設定すると上限を設けない。
    - 例：[move_base_params.yaml](https://gitlab.com/TeamSOBITS/sobit_navigation_stack/-/blob/main/sobit_navigation/param/sobit_turtlebot/move_base_params.yaml)

- planner_frequency   (double [Hz], default 0.0)
    - global plannerがグローバルパスを計算する頻度。0.0に設定すると、最初にゴールが設定された時のみにグローバルパスが計算される。
    - 例：[move_base_params.yaml](https://gitlab.com/TeamSOBITS/sobit_navigation_stack/-/blob/main/sobit_navigation/param/sobit_turtlebot/move_base_params.yaml)

詳しくはこちら
- [move_base でRecovery 行動に遷移する条件を調べてみた](https://qiita.com/MoriKen/items/1f1f2d1e6ef0046ec12a)
- [ROSのナビゲーションmove_baseについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-move_base/?amp)

# 参考サイト
- [ROSのナビゲーションmove_baseについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-move_base/?amp)
- [ロボットに固有なNavigationスタックのセットアップと環境設定](http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup)
- [global_path_planningを見直す](https://github.com/open-rdc/orne_navigation/issues/434)
- [Obstacle avoidance in move_base package](https://answers.ros.org/question/273029/obstacle-avoidance-in-move_base-package/)
- [move_base  obstacle avoidance: 3d vs 2d ](https://youtube.com/watch?v=a-5QgCqze3I&feature=share)
- [つくばチャレンジ 2019 における自律移動ロボット「Orange2019」の開発](https://robotgroup-soka.slack.com/archives/C028ZBNL17Z/p1631884367064200?thread_ts=1631884206.063600&channel=C028ZBNL17Z&message_ts=1631884367.064200 )
- [move_base でRecovery 行動に遷移する条件を調べてみた](https://qiita.com/MoriKen/items/1f1f2d1e6ef0046ec12a)
- [積み状態からの回復](https://qiita.com/MoriKen/items/d5cd6208143d6c40caff#%E7%A9%8D%E3%81%BF%E7%8A%B6%E6%85%8B%E3%81%8B%E3%82%89%E3%81%AE%E5%9B%9E%E5%BE%A9)

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)