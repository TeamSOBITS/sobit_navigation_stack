#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <atomic>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <time.h>

class KeyboardTeleop {
    private:
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Publisher pub_vel_;
        // double stack_time_;
        double accel_linear_;
        double accel_angular_;
        double vel_linear_;
        double vel_angular_;
        double timeout_sec_;
        std::string vel_topic_name_;
        bool isKeyPressed() {
            struct termios oldt, newt;
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);

            struct timeval tv;
            tv.tv_sec = (int)(timeout_sec_);
            tv.tv_usec = 1000000*(timeout_sec_ - (int)(timeout_sec_));

            int retval = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

            return (retval > 0);
        }
        double now_time_recognition() {
            struct timeval time_now {};
            gettimeofday(&time_now, nullptr);
            return ((double)(time_now.tv_sec) + ((double)(time_now.tv_usec) / 1000000));
        }
        geometry_msgs::Twist request_velocity(geometry_msgs::Twist vel, char input_mode, double range_time) {
            geometry_msgs::Twist new_vel;
            new_vel.linear.x = 0.;
            new_vel.linear.y = 0.;
            new_vel.linear.z = 0.;
            new_vel.angular.x = 0.;
            new_vel.angular.y = 0.;
            new_vel.angular.z = 0.;
            if ((input_mode == 'q') || (input_mode == 'z') || (input_mode == 'w') || (input_mode == 'x') || (input_mode == 'e') || (input_mode == 'c')) {
                if (input_mode == 'q') {
                    vel_linear_ = vel_linear_*1.1;
                    vel_angular_ = vel_angular_*1.1;
                }
                else if (input_mode == 'z') {
                    vel_linear_ = vel_linear_*0.9;
                    vel_angular_ = vel_angular_*0.9;
                }
                else if (input_mode == 'w') vel_linear_ = vel_linear_*1.1;
                else if (input_mode == 'x') vel_linear_ = vel_linear_*0.9;
                else if (input_mode == 'e') vel_angular_ = vel_angular_*1.1;
                else if (input_mode == 'c') vel_angular_ = vel_angular_*0.9;
                new_vel.linear.x = vel.linear.x;
                new_vel.linear.y = vel.linear.y;
                new_vel.linear.z = vel.linear.z;
                new_vel.angular.x = vel.angular.x;
                new_vel.angular.y = vel.angular.y;
                new_vel.angular.z = vel.angular.z;
                std::cout << std::endl << std::endl << std::endl << "Control Your Robot!" << std::endl << std::endl;
                std::cout << "---------------------------" << std::endl;
                std::cout << "Moving around:" << std::endl;
                std::cout << "   y    u    i    o    p" << std::endl;
                std::cout << "   h    j    k    l    ;" << std::endl;
                std::cout << "   n    m    ,    .    /" << std::endl << std::endl;
                std::cout << "q/z : increase/decrease max speeds by 10%" << std::endl;
                std::cout << "w/x : increase/decrease only linear speed by 10%" << std::endl;
                std::cout << "e/c : increase/decrease only angular speed by 10%" << std::endl << std::endl;
                std::cout << "space key, k : force stop" << std::endl;
                std::cout << "anything else : stop smoothly" << std::endl << std::endl;
                std::cout << "CTRL-C to quit" << std::endl << std::endl << std::endl;
                std::cout << "currently:	speed " << vel_linear_ << "	turn " << vel_angular_ << " " << std::endl << std::endl << std::endl << std::endl;
            }
            else if (((input_mode != 'k') && (input_mode != ' ')) || (input_mode == '\0')) {
                char current_mode;
                if (vel.linear.x < 0.) {
                    if (vel.linear.y < 0.) current_mode = '/';
                    else if (0. < vel.linear.y) current_mode = 'n';
                    else {
                        if (vel.angular.z < 0.) current_mode = 'm';
                        else if (0. < vel.angular.z) current_mode = '.';
                        else current_mode = ',';
                    }
                }
                else if (0. < vel.linear.x) {
                    if (vel.linear.y < 0.) current_mode = 'p';
                    else if (0. < vel.linear.y) current_mode = 'y';
                    else {
                        if (vel.angular.z < 0.) current_mode = 'o';
                        else if (0. < vel.angular.z) current_mode = 'u';
                        else current_mode = 'i';
                    }
                }
                else {
                    if (vel.linear.y < 0.) current_mode = ';';
                    else if (0. < vel.linear.y) current_mode = 'h';
                    else {
                        if (vel.angular.z < 0.) current_mode = 'l';
                        else if (0. < vel.angular.z) current_mode = 'j';
                        else current_mode = 'k';
                    }
                }
                if (((current_mode == input_mode) || (current_mode == 'k')) && (input_mode != '\0')) {
                    if (input_mode == 'y') {
                        if (std::sqrt(std::pow(vel.linear.x + (1/std::sqrt(2.)) * accel_linear_*range_time, 2.) + std::pow(vel.linear.y + (1/std::sqrt(2.)) * accel_linear_*range_time, 2.)) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x + (1/std::sqrt(2.)) * accel_linear_*range_time;
                            new_vel.linear.y = vel.linear.y + (1/std::sqrt(2.)) * accel_linear_*range_time;
                        }
                        else {
                            new_vel.linear.x =  (1/std::sqrt(2.)) * vel_linear_;
                            new_vel.linear.y =  (1/std::sqrt(2.)) * vel_linear_;
                        }
                    }
                    else if (input_mode == 'p') {
                        if (std::sqrt(std::pow(vel.linear.x + (1/std::sqrt(2.)) * accel_linear_*range_time, 2.) + std::pow(vel.linear.y - (1/std::sqrt(2.)) * accel_linear_*range_time, 2.)) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x + (1/std::sqrt(2.)) * accel_linear_*range_time;
                            new_vel.linear.y = vel.linear.y - (1/std::sqrt(2.)) * accel_linear_*range_time;
                        }
                        else {
                            new_vel.linear.x =  (1/std::sqrt(2.)) * vel_linear_;
                            new_vel.linear.y = -(1/std::sqrt(2.)) * vel_linear_;
                        }
                    }
                    else if (input_mode == 'n') {
                        if (std::sqrt(std::pow(vel.linear.x - (1/std::sqrt(2.)) * accel_linear_*range_time, 2.) + std::pow(vel.linear.y + (1/std::sqrt(2.)) * accel_linear_*range_time, 2.)) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x - (1/std::sqrt(2.)) * accel_linear_*range_time;
                            new_vel.linear.y = vel.linear.y + (1/std::sqrt(2.)) * accel_linear_*range_time;
                        }
                        else {
                            new_vel.linear.x = -(1/std::sqrt(2.)) * vel_linear_;
                            new_vel.linear.y =  (1/std::sqrt(2.)) * vel_linear_;
                        }
                    }
                    else if (input_mode == '/') {
                        if (std::sqrt(std::pow(vel.linear.x - (1/std::sqrt(2.)) * accel_linear_*range_time, 2.) + std::pow(vel.linear.y - (1/std::sqrt(2.)) * accel_linear_*range_time, 2.)) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x - (1/std::sqrt(2.)) * accel_linear_*range_time;
                            new_vel.linear.y = vel.linear.y - (1/std::sqrt(2.)) * accel_linear_*range_time;
                        }
                        else {
                            new_vel.linear.x = -(1/std::sqrt(2.)) * vel_linear_;
                            new_vel.linear.y = -(1/std::sqrt(2.)) * vel_linear_;
                        }
                    }
                    else if (input_mode == 'u') {
                        double r = vel_linear_ / vel_angular_;
                        if (std::fabs(vel.linear.x + accel_linear_*range_time) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x + accel_linear_*range_time;
                            new_vel.angular.z =  std::fabs(new_vel.linear.x) / r;
                        }
                        else {
                            new_vel.linear.x =  vel_linear_;
                            new_vel.angular.z =  vel_angular_;
                        }
                    }
                    else if (input_mode == 'm') {
                        double r = vel_linear_ / vel_angular_;
                        if (std::fabs(vel.linear.x - accel_linear_*range_time) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x - accel_linear_*range_time;
                            new_vel.angular.z = -std::fabs(new_vel.linear.x) / r;
                        }
                        else {
                            new_vel.linear.x = -vel_linear_;
                            new_vel.angular.z = -vel_angular_;
                        }
                    }
                    else if (input_mode == 'o') {
                        double r = vel_linear_ / vel_angular_;
                        if (std::fabs(vel.linear.x + accel_linear_*range_time) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x + accel_linear_*range_time;
                            new_vel.angular.z = -std::fabs(new_vel.linear.x) / r;
                        }
                        else {
                            new_vel.linear.x =  vel_linear_;
                            new_vel.angular.z = -vel_angular_;
                        }
                    }
                    else if (input_mode == '.') {
                        double r = vel_linear_ / vel_angular_;
                        if (std::fabs(vel.linear.x - accel_linear_*range_time) <= vel_linear_) {
                            new_vel.linear.x = vel.linear.x - accel_linear_*range_time;
                            new_vel.angular.z =  std::fabs(new_vel.linear.x) / r;
                        }
                        else {
                            new_vel.linear.x = -vel_linear_;
                            new_vel.angular.z =  vel_angular_;
                        }
                    }
                    else if (input_mode == 'i') {
                        if (std::fabs(vel.linear.x + accel_linear_*range_time) <= vel_linear_) new_vel.linear.x = vel.linear.x + accel_linear_*range_time;
                        else new_vel.linear.x = vel_linear_;
                    }
                    else if (input_mode == ',') {
                        if (std::fabs(vel.linear.x - accel_linear_*range_time) <= vel_linear_) new_vel.linear.x = vel.linear.x - accel_linear_*range_time;
                        else new_vel.linear.x = -vel_linear_;
                    }
                    else if (input_mode == 'h') {
                        if (std::fabs(vel.linear.y + accel_linear_*range_time) <= vel_linear_) new_vel.linear.y = vel.linear.y + accel_linear_*range_time;
                        else new_vel.linear.y = vel_linear_;
                    }
                    else if (input_mode == ';') {
                        if (std::fabs(vel.linear.y - accel_linear_*range_time) <= vel_linear_) new_vel.linear.y = vel.linear.y - accel_linear_*range_time;
                        else new_vel.linear.y = -vel_linear_;
                    }
                    else if (input_mode == 'j') {
                        if (std::fabs(vel.angular.z + accel_angular_*range_time) <= vel_linear_) new_vel.angular.z = vel.angular.z + accel_angular_*range_time;
                        else new_vel.angular.z =  vel_angular_;
                    }
                    else if (input_mode == 'l') {
                        if (std::fabs(vel.angular.z - accel_angular_*range_time) <= vel_linear_) new_vel.angular.z = vel.angular.z - accel_angular_*range_time;
                        else new_vel.angular.z = -vel_angular_;
                    }
                }
                else {
                    if (std::fabs(vel.angular.z) == 0.) {
                        double angle = std::atan2(vel.linear.y, vel.linear.x);
                        if ((std::sqrt(std::pow(vel.linear.x, 2.) + std::pow(vel.linear.y, 2.)) - accel_linear_*range_time) < 0.) {
                            new_vel.linear.x = 0.;
                            new_vel.linear.y = 0.;
                        }
                        else {
                            new_vel.linear.x = (std::sqrt(std::pow(vel.linear.x, 2.) + std::pow(vel.linear.y, 2.)) - accel_linear_*range_time) * std::cos(angle);
                            new_vel.linear.y = (std::sqrt(std::pow(vel.linear.x, 2.) + std::pow(vel.linear.y, 2.)) - accel_linear_*range_time) * std::sin(angle);
                        }
                    }
                    else {
                        double r = vel_linear_ / vel_angular_;
                        if ((std::fabs(vel.linear.x) - accel_linear_*range_time) < 0.) new_vel.linear.x = 0.;
                        else {
                            new_vel.linear.x = (std::fabs(vel.linear.x) - accel_linear_*range_time) * (vel.linear.x/std::fabs(vel.linear.x));
                            new_vel.angular.z = (std::fabs(new_vel.linear.x) / r) * (vel.angular.z/std::fabs(vel.angular.z));
                        }
                    }
                }
            }
            return new_vel;
        }
    public:
        KeyboardTeleop() : nh_(), pnh_("~") {
            timeout_sec_ = pnh_.param<double>("timeout_sec", 0.3);
            vel_topic_name_ = pnh_.param<std::string>("velocity_topic_name", "/mobile_base/commands/velocity");
            vel_linear_ = pnh_.param<double>("max_linear", 0.2);
            vel_angular_ = pnh_.param<double>("max_angular", 0.7);
            accel_linear_ = pnh_.param<double>("accel_linear", 0.3);
            accel_angular_ = pnh_.param<double>("accel_angular", 0.7);
            pub_vel_ = nh_.advertise<geometry_msgs::Twist>( vel_topic_name_ , 1);
            loop_function();
        }
        void loop_function() {
            std::cout << std::endl << "Control Your Robot!" << std::endl << std::endl;
            std::cout << "---------------------------" << std::endl;
            std::cout << "Moving around:" << std::endl;
            std::cout << "   y    u    i    o    p" << std::endl;
            std::cout << "   h    j    k    l    ;" << std::endl;
            std::cout << "   n    m    ,    .    /" << std::endl << std::endl;
            std::cout << "q/z : increase/decrease max speeds by 10%" << std::endl;
            std::cout << "w/x : increase/decrease only linear speed by 10%" << std::endl;
            std::cout << "e/c : increase/decrease only angular speed by 10%" << std::endl << std::endl;
            std::cout << "space key, k : force stop" << std::endl;
            std::cout << "anything else : stop smoothly" << std::endl << std::endl;
            std::cout << "CTRL-C to quit" << std::endl << std::endl << std::endl;
            std::cout << "currently:	speed " << vel_linear_ << "	turn " << vel_angular_ << " " << std::endl << std::endl << std::endl << std::endl;

            geometry_msgs::Twist vel, stack_vel;
            vel.linear.x = 0.;
            vel.linear.y = 0.;
            vel.linear.z = 0.;
            vel.angular.x = 0.;
            vel.angular.y = 0.;
            vel.angular.z = 0.;
            stack_vel.linear.x = 0.;
            stack_vel.linear.y = 0.;
            stack_vel.linear.z = 0.;
            stack_vel.angular.x = 0.;
            stack_vel.angular.y = 0.;
            stack_vel.angular.z = 0.;
            // char stack_str = '\0';
            char current_str;
            double now_time = now_time_recognition();
            double stack_time = now_time_recognition();
            while (ros::ok()) {
                if (isKeyPressed()) std::cin.get(current_str);
                else current_str = '\0';
                now_time = now_time_recognition();
                vel = request_velocity(stack_vel, current_str, now_time - stack_time);
                pub_vel_.publish(vel);
                stack_vel.linear.x = vel.linear.x;
                stack_vel.linear.y = vel.linear.y;
                stack_vel.linear.z = vel.linear.z;
                stack_vel.angular.x = vel.angular.x;
                stack_vel.angular.y = vel.angular.y;
                stack_vel.angular.z = vel.angular.z;
                stack_time = now_time_recognition();
            }
        }
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "keyboard_teleop");
    KeyboardTeleop kt;
	ros::spin();
	return 0;
}
