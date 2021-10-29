#ifndef SOBIT_NAVIGATION_LIBRARY
#define SOBIT_NAVIGATION_LIBRARY

// Ref : ROS Navigation Stack について3 ~ Goalを送信するノードの作成 ~ ( https://daily-tech.hatenablog.com/entry/2017/02/11/214336 )
// Ref : ROS講座95 actionlibを使う( https://qiita.com/srs/items/a39dcd24aaeb03216026 )
// move_base_msgs/MoveBaseAction Message : http://docs.ros.org/en/diamondback/api/move_base_msgs/html/msg/MoveBaseAction.html
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalStatusArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace SOBITNavigationStack {
    class LocationPose {
        public :
            std::string name;
            geometry_msgs::Pose pose;
    };

    class SOBITNavigationLibrary {
        protected :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            MoveBaseClient act_clt_;
            ros::Subscriber sub_status_;

            bool is_output_;
            move_base_msgs::MoveBaseGoal goal_;
            std::vector<std::string> status_name_ = { 
                "PENDING",  
                "ACTIVE",
                "PREEMPTED",
                "SUCCEEDED",
                "ABORTED",
                "REJECTED",
                "PREEMPTING",
                "RECALLING",
                "RECALLED",
                "LOST"
            };

            // ロケーションファイルを読み込む関数
            void loadLocationFile();
            // 目的位置をアクションサーバーに送る関数
            void sendGoal( void );
            // アクションサービス完了後の結果を返すコールバック関数
            void doneCb(
                const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr& result );
            // アクションサーバーが動作しているかを返すコールバック関数
            void activeCb( void );
            // アクションサーバーのフィードバックを返すコールバック関数
            void feedbackCb( const move_base_msgs::MoveBaseFeedbackConstPtr& feedback );
            // アクションサーバーのステータスを返すコールバック関数
            void statusCb(const actionlib_msgs::GoalStatusArray::ConstPtr &status);

        public :
            // ロケーションポーズの配列
            std::vector<LocationPose> location_poses_;
            // フラグ
            bool exist_goal_;
            int status_id_;

            // コンストラクタ
            SOBITNavigationLibrary();
            
            // 移動したい位置(geometry_msgs::Pose型)に移動する
            bool move2Position(
                const geometry_msgs::Pose& target_position,
                const std::string& frame_id,
                const bool is_wait = false  );
            
            // 移動したい位置に移動する(Pybind用)
            bool move2Position(
                const double x,
                const double y,
                const double z,
                const double qx,
                const double qy,
                const double qz,
                const double qw,
                const std::string& frame_id,
                const bool is_wait = false );

            // ロケーションファイルの位置(std::string型)に移動する
            bool move2Location(
                const std::string&  location_name,
                const bool is_wait = false  );

            // 移動をキャンセルする(アクションサーバの処理を中断する)
            void cancelMoving();
    };
}

#endif
