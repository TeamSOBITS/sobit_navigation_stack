# About SOBIT Navigation Library
ナビゲーションをプログラム上で実行できるライブラリ

# SOBIT Navigation Library([.hpp](sobit_navigation_library/include/sobit_navigation_library/sobit_navigation_library.hpp), [.cpp](sobit_navigation_library/src/sobit_navigation_library.cpp))
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
void clearCostmaps()
```

# SOBIT Navigation Library Python ([.hpp](sobit_navigation_library/include/sobit_navigation_library/sobit_navigation_library_python.hpp), [.cpp](sobit_navigation_library/src/sobit_navigation_library_python.cpp))
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

### clearCostmaps
- コストマップのクリア
```cpp
void clearCostmaps()
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

## How to Use
### C++
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
### Python
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

※以下のエラーが出力された場合：「#!/usr/bin/env python3」→「#!/usr/bin/env python」
```bash
Traceback (most recent call last):
  File "/home/sobits/catkin_ws/src/nav_test/script/test.py", line 3, in <module>
    from sobit_navigation_module import SOBITNavigationLibraryPython
ImportError: dynamic module does not define module export function (PyInit_sobit_navigation_module)
```

---

- [Topに戻る](https://gitlab.com/TeamSOBITS/sobit_navigation_stack#sobit-navigation-stack)