#ifndef SOBIT_NAVIGATION_LIBRARY
#define SOBIT_NAVIGATION_LIBRARY

// Ref : ROS Navigation Stack について3 ~ Goalを送信するノードの作成 ~ ( https://daily-tech.hatenablog.com/entry/2017/02/11/214336 )
// Ref : ROS講座95 actionlibを使う( https://qiita.com/srs/items/a39dcd24aaeb03216026 )

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace SOBITNavigationStack {
    class SOBITNavigationLibrary {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            MoveBaseClient act_clt_;
            bool is_output_;

            // 目的位置をアクションサーバーに送る関数
            virtual void sendGoal( const move_base_msgs::MoveBaseGoal& goal );

        public :
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

            // アクションサービス完了後の結果を返すコールバック関数
            virtual void doneCb(
                const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseActionResultConstPtr& result );
            
            // アクションサーバーが動作しているかを返すコールバック関数
            virtual void activeCb( );
            
            // アクションサーバーのフィードバックを返すコールバック関数
            virtual void feedbackCb(
                const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback );
    };
}

#endif
