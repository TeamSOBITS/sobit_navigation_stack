#include <sobit_navigation_library/sobit_navigation_library.hpp>

using namespace SOBITNavigationStack;

// ロケーションファイルを読み込む関数
void SOBITNavigationLibrary::loadLocationFile() {
    XmlRpc::XmlRpcValue pose_val;
    if ( nh_.hasParam("/location_pose") ) ROS_INFO( "[ SOBITNavigationLibrary ] Load the location file\n" );
    else {
        ROS_ERROR( "[ SOBITNavigationLibrary ] The location file path does not exist.\n" );
        return;
    }

    nh_.getParam("/location_pose", pose_val);
    int pose_num = pose_val.size();
    location_poses_.clear();
    for ( int i = 0; i < pose_num; i++ ) {
        LocationPose pose;
        pose.name = static_cast<std::string>(pose_val[i]["location_name"]); 
        pose.frame_id = static_cast<std::string>(pose_val[i]["frame_id"]); 
        pose.pose.position.x = static_cast<double>(pose_val[i]["translation_x"]); 
        pose.pose.position.y = static_cast<double>(pose_val[i]["translation_y"]); 
        pose.pose.position.z = static_cast<double>(pose_val[i]["translation_z"]); 
        pose.pose.orientation.x = static_cast<double>(pose_val[i]["rotation_x"]); 
        pose.pose.orientation.y = static_cast<double>(pose_val[i]["rotation_y"]); 
        pose.pose.orientation.z = static_cast<double>(pose_val[i]["rotation_z"]); 
        pose.pose.orientation.w = static_cast<double>(pose_val[i]["rotation_w"]); 
        location_poses_.push_back( pose );
    }
    return;
}
// 目的位置をアクションサーバーに送る関数
void SOBITNavigationLibrary::sendGoal( void  ) {
    ROS_INFO( "[ SOBITNavigationLibrary::sendGoal ] Sending goal\n" );
    act_clt_.sendGoal( goal_,
        boost::bind( &SOBITNavigationLibrary::doneCb, this, _1, _2 ),
        boost::bind( &SOBITNavigationLibrary::activeCb, this ),
        boost::bind( &SOBITNavigationLibrary::feedbackCb, this, _1 ) );
    return;
}
// アクションサービス完了後の結果を返すコールバック関数
void SOBITNavigationLibrary::doneCb( const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result ) {
    if ( is_output_ ) {
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO( "[ SOBITNavigationLibrary::doneCb ] Goal reached!!\n" );
        else ROS_ERROR( "[ SOBITNavigationLibrary::doneCb ] Goal failed!!\n" );
    }
    exist_goal_ = false;
    return;
}
// アクションサーバーが動作しているかを返すコールバック関数
void SOBITNavigationLibrary::activeCb( void ) {
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary::activeCb ] Goal just went active...\n" );
    return;
}
// アクションサーバーのフィードバックを返すコールバック関数
void SOBITNavigationLibrary::feedbackCb( const move_base_msgs::MoveBaseFeedbackConstPtr& feedback ) {
    if ( is_output_ ) {
        ROS_INFO( "[ SOBITNavigationLibrary::feedbackCb ] Got Feedback of Progress to Goal" );
        std::cout << "[ Feedback Info ]" 
            << "\nStatus ID       : " << status_name_[ status_id_ ] << " ( id = " << status_id_ << " )"
            << "\nBase Pose       :  x = " << feedback->base_position.pose.position.x << " [m], y = " << feedback->base_position.pose.position.y << " [m]"
            << "\nTarget distance : " << std::hypotf( goal_.target_pose.pose.position.x - feedback->base_position.pose.position.x, goal_.target_pose.pose.position.y - feedback->base_position.pose.position.y )
            << "\n" << std::endl;
    }
    return;
}
// アクションサーバーのステータスを返すコールバック関数
void SOBITNavigationLibrary::statusCb(const actionlib_msgs::GoalStatusArray::ConstPtr &status) {
    if (!status->status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
        if ( is_output_ && status_id_ != goalStatus.status ) {
            ROS_INFO( "[ SOBITNavigationLibrary::statusCb ] Got Status : status = %s (id = %d)", status_name_[ goalStatus.status ].c_str(), goalStatus.status );
        }
        status_id_ = goalStatus.status;
    }
}

// コンストラクタ
SOBITNavigationLibrary::SOBITNavigationLibrary() : nh_(), pnh_("~"),  act_clt_( "move_base", true ) {
    is_output_ = pnh_.param<bool>( "is_output_terminal", true );
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary ] Waiting For Server...\n" );
    act_clt_.waitForServer();
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary ] Connect to the action server\n" );
    sub_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &SOBITNavigationLibrary::statusCb, this);
    clt_clear_costmaps_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    pub_pose_estimate_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    loadLocationFile();
    exist_goal_ = false;
    status_id_ = 0;
}
SOBITNavigationLibrary::SOBITNavigationLibrary( const std::string &name ) :  ROSCommonNode( name ), nh_(), pnh_("~"), act_clt_( "move_base", true ) {
    is_output_ = pnh_.param<bool>( "is_output_terminal", true );
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary ] Waiting For Server...\n" );
    act_clt_.waitForServer();
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary ] Connect to the action server\n" );
    sub_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &SOBITNavigationLibrary::statusCb, this);
    clt_clear_costmaps_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    pub_pose_estimate_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    loadLocationFile();
    exist_goal_ = false;
    status_id_ = 0;
}

// 移動したい位置(geometry_msgs::Pose型)に移動する
bool SOBITNavigationLibrary::move2Position( const geometry_msgs::Pose& target_position, const std::string& frame_id, const bool is_wait  ) {
    if ( is_output_ )  {
        ROS_INFO( "[ SOBITNavigationLibrary::move2Position ] Set a new Goal" );
        std::cout << "[ New Goal Info ]"
            << "\nframe_id : " << frame_id
            << "\nPose     : x = " << target_position.position.x << "[m], y = " << target_position.position.y << " [m]"
            << "\n" << std::endl;
    }

    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = target_position;
    goal_ = goal;
    exist_goal_ = true;
    sendGoal( );
    if ( is_wait ) act_clt_.waitForResult();
    return true;
}

// ロケーションファイルの位置(std::string型)に移動する
bool SOBITNavigationLibrary::move2Location( const std::string&  location_name, const bool is_wait  ) {
    geometry_msgs::Pose target_position;
    std::string frame_id;
    bool exist_target = false;
    for ( const auto pose : location_poses_ ) {
        if ( location_name != pose.name ) continue;
        target_position = pose.pose;
        frame_id = pose.frame_id;
        exist_target = true;
    }
    if ( !exist_target ) {
        ROS_ERROR( "[ SOBITNavigationLibrary::move2Location ] \"%s\" does not exist.\n", location_name.c_str() );
        return false;
    }
    move2Position( target_position, frame_id, is_wait );
    return true;
}

// 移動をキャンセルする(アクションサーバの処理を中断する)
void SOBITNavigationLibrary::cancelMoving( ) {
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary::cancelMoving ] Cancel the move\n" );
    act_clt_.cancelGoal();
    exist_goal_ = false;
    return;
}


// ロケーションポジションの追加
void SOBITNavigationLibrary::addLocationPose( const std::string& name, const std::string& frame_id, const geometry_msgs::Pose& target_position ) {
    if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary::addLocationPose ] Add a location position\n" );
    LocationPose pose;
    pose.name = name;
    pose.frame_id = frame_id;
    pose.pose = target_position;
    location_poses_.push_back( pose );
    return;
}

// コストマップのクリア
void SOBITNavigationLibrary::clearCostmaps() {
    std_srvs::Empty srv;
    clt_clear_costmaps_.call(srv);
    return;
}

// 現在地を指定したロケーションポーズの位置にする
void SOBITNavigationLibrary::estimatePoseFromLocation( const std::string&  location_name ) {
    geometry_msgs::PoseWithCovarianceStamped target_position;
    std::string frame_id;
    bool exist_target = false;
    for ( const auto pose : location_poses_ ) {
        if ( location_name != pose.name ) continue;
        target_position.pose.pose = pose.pose;
        target_position.header.frame_id = pose.frame_id;
        target_position.header.stamp = ros::Time::now();
        exist_target = true;
    }
    if ( !exist_target ) {
        ROS_ERROR( "[ SOBITNavigationLibrary::move2Location ] \"%s\" does not exist.\n", location_name.c_str() );
        return;
    }    
    pub_pose_estimate_.publish(target_position);
}