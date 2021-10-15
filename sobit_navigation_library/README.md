# About SOBIT Navigation Library
ナビゲーションをプログラム上で実行できるライブラリ

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

### move2Position
- 移動したい位置に移動する(Pybind用)
```cpp
bool SOBITNavigationStack::SOBITNavigationLibrary::move2Position( 
    const float64 x,                // 位置
    const float64 y,                // 位置 
    const float64 z,                // 位置
    const float64 qx,               // 姿勢
    const float64 qy,               // 姿勢 
    const float64 qz,               // 姿勢 
    const float64 qw,               // 姿勢 
    const std::string& frame_id,    // 基準フレーム
    const bool is_wait = false      // 移動完了までプログラムを待機させるか
);               
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

### sendGoal
- 目的位置をアクションサーバーに送る関数
```cpp
virtual void SOBITNavigationStack::SOBITNavigationLibrary::sendGoal( )            
```

### doneCb
- アクションサービス完了後の結果を返すコールバック関数
```cpp
virtual void SOBITNavigationStack::SOBITNavigationLibrary::doneCb(
    const actionlib::SimpleClientGoalState& state, 
    const move_base_msgs::MoveBaseActionResultConstPtr& result 
)            
```

### activeCb
- アクションサーバーが動作しているかを返すコールバック関数
```cpp
virtual void SOBITNavigationStack::SOBITNavigationLibrary::activeCb( )            
```

### feedbackCb
- アクションサーバーのフィードバックを返すコールバック関数
```cpp
virtual void SOBITNavigationStack::SOBITNavigationLibrary::feedbackCb( 
    const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback 
)            
```

## How to Use
### C++

### Python