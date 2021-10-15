#include <sobit_navigation_library/sobit_navigation_library.hpp>

using namespace SOBITNavigationStack;

// 目的位置をアクションサーバーに送る関数
void SOBITNavigationLibrary::sendGoal( const move_base_msgs::MoveBaseGoal& goal  ) {
    ROS_INFO( "[ %s ] Sending goal\n", ros::this_node::getName().c_str() );
    return;
}

// コンストラクタ
SOBITNavigationLibrary::SOBITNavigationLibrary() : nh_(), pnh_("~"),  act_clt_( "move_base", true ) {
    is_output_ = pnh_.param<bool>( "is_output_terminal", true );
    if ( is_output_ ) ROS_INFO( "[ %s ] Waiting For Server...\n", ros::this_node::getName().c_str() );
    act_clt_.waitForServer();
    if ( is_output_ ) ROS_INFO( "[ %s ] Connect to the action server\n", ros::this_node::getName().c_str() );
}

// 移動したい位置(geometry_msgs::Pose型)に移動する
bool SOBITNavigationLibrary::move2Position( const geometry_msgs::Pose& target_position, const std::string& frame_id, const bool is_wait  ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Set a new Goal\n", ros::this_node::getName().c_str() );
    return true;
}

// 移動したい位置に移動する(Pybind用)
bool SOBITNavigationLibrary:: move2Position(
    const double x,
    const double y,
    const double z,
    const double qx,
    const double qy,
    const double qz,
    const double qw,
    const std::string& frame_id,
    const bool is_wait ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Set a new Goal\n", ros::this_node::getName().c_str() );
    return true;
}

// ロケーションファイルの位置(std::string型)に移動する
bool SOBITNavigationLibrary::move2Location( const std::string&  location_name, const bool is_wait  ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Set a new Goal\n", ros::this_node::getName().c_str() );
    return true;
}

// 移動をキャンセルする(アクションサーバの処理を中断する)
void SOBITNavigationLibrary::cancelMoving( ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Cancel the move\n", ros::this_node::getName().c_str() );
    return;
}

// アクションサービス完了後の結果を返すコールバック関数
void SOBITNavigationLibrary::doneCb( const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseActionResultConstPtr& result ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Finished!!\n", ros::this_node::getName().c_str() );
    return;
}

// アクションサーバーが動作しているかを返すコールバック関数
void SOBITNavigationLibrary::activeCb( ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Goal just went active...\n", ros::this_node::getName().c_str() );
    return;
}

// アクションサーバーのフィードバックを返すコールバック関数
void SOBITNavigationLibrary::feedbackCb( const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback ) {
    if ( is_output_ ) ROS_INFO( "[ %s ] Got Feedback of Progress to Goal\n", ros::this_node::getName().c_str() );
    return;
}



