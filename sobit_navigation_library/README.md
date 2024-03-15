# NavigationをActionlibによってプログラムから呼び出すためのパッケージ
これはライブラリなので，基本的に他のROSパッケージから利用します．\
普通に移動するなら，exampleコードの[move_location_example.py(python用)](/sobit_navigation_library/example/move_location_example.py)，[move_location_example.cpp(C++用)](/sobit_navigation_library/example/move_location_example.cpp)を使う．\
以下に，詳しいツールについて紹介する．

> [!NOTE]
> 別のパッケージからC++で呼び出す場合は，そのパッケージのCMakeLists.txtと，package.xmlに依存を追加しなければならない．
> CMakeLists.txtのfind_package()のカッコ内に"sobit_navigation_library"という行を追加．
> package.xmlの内に```<depend>sobit_navigation_library</depend>```というタグの行を追加．

## Navigationツール

- 座標を直接指定したゴールの送り方
    メインのREADMEでは地点登録をした位置に移動する方法について書い．\
    しかし移動先の位置がどこなのか，地点登録によらない場合がある．\
    例えば，未知環境でのレストランタスクでは，手をあげた人を見つけ，その位置まで移動する．\
    その場合，当然事前に地点登録ができるわけがなく，「手を挙げた人の座標」にゴール送必要がある．
    そこで，そのようなゴールを送ることのできプログラム[sobit_navigation_library/example/move_position_example.py](/sobit_navigation_library/example/move_position_example.py)を起動する．




-----------------------------以下修正前のもの-----------------------------

# About SOBIT Navigation Library
ナビゲーションをプログラム上で実行できるライブラリ

# Setup
これはライブラリなので，基本的に他のROSパッケージから利用します．  
そこでライブラリを使用したいパッケージでSOBIT Navigation Libraryインクルードする必要があります．  
まず，CMakeLists.txt （ライブラリを使用したいパッケージのもの）の中に、以下を追加してください．  

```python
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sobit_navigation_library
)
```

そして、package.xmlの中に```<depend>sobit_navigation_library</depend>```タグを追加してください。

こうすることで、依存関係を適切に設定することができます。

これで、catkin_makeでコンパイルすれば完了です．

## How to use
1. 以下のどれかのnavigationを起動
```python
$ roslaunch sobit_navigation sobit_turtlebot_navigation.launch
# $ roslaunch sobit_navigation sobit_turtlebot_navigation_octomap.launch
$ roslaunch sobit_navigation sobit_pro_navigation.launch
# $ roslaunch sobit_navigation sobit_pro_navigation_octomap.launch
```

2. [load_location_file](sobit_mapping/launch/load_location_file.launch)を起動
```python
roslaunch sobit_mapping load_location_file.launch 
```
※ location_fileのパス設定を間違えないよう注意

3.　下記のライブラリが使用できる

# Example Code
<details><summary>C++</summary>

## C++
```cpp
#include <ros/ros.h>
#include <sobit_navigation_library/sobit_navigation_library.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_include_library");
    ros::NodeHandle nh;
    SOBITNavigationStack::SOBITNavigationLibrary nav_lib;
    nav_lib.move2Location( "table_1", false );

    double start_time = ros::Time::now().toSec();
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        double curt_time = ros::Time::now().toSec();
        double elapsed_time = curt_time - start_time;
        if( elapsed_time > 10.0 ) {
            nav_lib.cancelMoving();
            nav_lib.move2Location( "init", false );
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
```
</details>

<details><summary>Python</summary>

## Python

```py
#!/usr/bin/env python3
import rospy
from sobit_navigation_module import SOBITNavigationLibraryPython
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    ang = 0.8
    args = sys.argv
    nav_lib = SOBITNavigationLibraryPython(args[0]) # args[0] : C++上でros::init()を行うための引数

    nav_lib.move2Location( "table_1", False )
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("move2Location")
        r.sleep()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

```

</details>

# SOBIT Navigation Library([.hpp](../sobit_navigation_library/include/sobit_navigation_library/sobit_navigation_library.hpp), [.cpp](../sobit_navigation_library/src/sobit_navigation_library.cpp))
- C++でナビゲーションの命令を送ることができるライブラリ

<details><summary>Member variables</summary>

## Member variables
### location_poses_
- ロケーションポーズ配列
```cpp
std::vector<LocationPose> location_poses_
```
- LocationPose
```cpp
class LocationPose {
    public :
        std::string name;
        geometry_msgs::Pose pose;
};
```

### exist_goal_
- ゴールが設定されているか
```cpp
bool exist_goal_
```

### status_id_
- MoveBaseの現在ステータス
```cpp
int status_id_
```

### result_
- 直近のナビゲーションの結果
```cpp
bool result_
```

</details>

<details><summary>Functions</summary>

## Functions
### move2Position
- 移動したい位置(geometry_msgs::Pose型)に移動する
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::move2Position( 
    const geometry_msgs::Pose& target_position,     // 目的位置(geometry_msgs::Pose型)
    const std::string& frame_id                     // 基準フレーム
    const bool is_wait = false                      // 移動完了までプログラムを待機させるか
)                   
```

### move2Location
- ロケーションファイルにある位置(std::string型)に移動する
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::move2Location( 
    const std::string&  location_name,  // ロケーションファイルにある位置(std::string型)
    const bool is_wait = false          // 移動完了までプログラムを待機させるか
)            
```

### cancelMoving
- 移動をキャンセルする(アクションサーバの処理を中断する)
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::cancelMoving( )            
```

### addLocationPose
- ロケーションポジションの追加
```cpp
void addLocationPose( 
    const std::string& name, 
    const std::string& frame_id, 
    const geometry_msgs::Pose& target_position 
)
```
### clearCostmaps
- コストマップのクリア
```cpp
void SOBITNavigationStack::clearCostmaps()
```

### estimatePoseFromLocation
- 現在地を指定したロケーションポーズの位置にする
```cpp
void SOBITNavigationLibrary::estimatePoseFromLocation( const std::string&  location_name )
```

</details>

# SOBIT Navigation Library Python ([.hpp](../sobit_navigation_library/include/sobit_navigation_library/sobit_navigation_library_python.hpp), [.cpp](../sobit_navigation_library/src/sobit_navigation_library_python.cpp))
- Pythonでナビゲーションの命令を送ることができるライブラリ

<details><summary>Member variables</summary>

## Member variables
### exist_goal_
- ゴールが設定されているか
```cpp
bool exist_goal_
```
### status_id_
- MoveBaseの現在ステータス
```cpp
int status_id_
```
### result_
- 直近のナビゲーションの結果
```cpp
bool result_
```

</details>

<details><summary>Functions</summary>

## Functions

### move2Position
- 移動したい位置に移動する(Pybind用)
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::move2PositionPy( 
    const float64 x,                // 位置
    const float64 y,                // 位置 
    const float64 z,                // 位置
    const float64 qx,               // 姿勢
    const float64 qy,               // 姿勢 
    const float64 qz,               // 姿勢 
    const float64 qw,               // 姿勢 
    const std::string& frame_id,    // 基準フレーム
    const bool is_wait = false      // 移動完了までプログラムを待機させるか
)          
```
### move2Location
- ロケーションファイルにある位置(std::string型)に移動する
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::move2Location( 
    const std::string&  location_name,  // ロケーションファイルにある位置(std::string型)
    const bool is_wait = false          // 移動完了までプログラムを待機させるか
)            
```
### cancelMoving
- 移動をキャンセルする(アクションサーバの処理を中断する)
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::cancelMoving( )            
```

### addLocationPosePy
- ロケーションポジションの追加(Pybind用)
```cpp
void addLocationPosePy( 
    const std::string& name, 
    const std::string& frame_id, 
    const float64 x,                // 位置
    const float64 y,                // 位置 
    const float64 z,                // 位置
    const float64 qx,               // 姿勢
    const float64 qy,               // 姿勢 
    const float64 qz,               // 姿勢 
    const float64 qw,               // 姿勢 
)
```

### clearCostmaps
- コストマップのクリア
```cpp
void SOBITNavigationStack::clearCostmaps()
```

### estimatePoseFromLocation
- 現在地を指定したロケーションポーズの位置にする
```cpp
void SOBITNavigationLibrary::estimatePoseFromLocation( const std::string&  location_name )
```

</details>

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)
