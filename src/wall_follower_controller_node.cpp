#include <ros/ros.h>
#include <cmath>

#include <s8_common_node/Node.h>
#include <s8_wall_follower_controller/wall_follower_controller_node.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Twist.h>
#include <s8_msgs/IRDistances.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>

#define HZ                              10

#define PARAM_FOLLOWING_KP_NAME         "following_kp"
#define PARAM_FOLLOWING_KP_DEFAULT      3.5
#define PARAM_FOLLOWING_KD_NAME         "following_kd"
#define PARAM_FOLLOWING_KD_DEFAULT      6.0
#define PARAM_FOLLOWING_KI_NAME         "following_ki"
#define PARAM_FOLLOWING_KI_DEFAULT      0.4
#define PARAM_KP_NEAR_NAME              "kp_near"
#define PARAM_KP_NEAR_DEFAULT           0.9
#define PARAM_KP_FAR_NAME               "kp_far"
#define PARAM_KP_FAR_DEFAULT            0.1
#define PARAM_DISTANCE_NAME             "distance"
#define PARAM_DISTANCE_DEFAULT          0.1
#define PARAM_I_THRESHOLD_NAME          "i_threshold"
#define PARAM_I_THRESHOLD_DEFAULT       0.02
#define PARAM_I_LIMIT_NAME              "i_limit"
#define PARAM_I_LIMIT_DEFAULT           0.02
#define PARAM_LINEAR_SPEED_NAME         "linear_speed"
#define PARAM_LINEAR_SPEED_DEFAULT      0.2
#define PARAM_IR_THRESHOLD_NAME         "ir_threshold"
#define PARAM_IR_THRESHOLD_DEFAULT      0.2
#define PARAM_ALIGNMENT_KP_NAME         "alignment_kp"
#define PARAM_ALIGNMENT_KP_DEFAULT      4.0
#define PARAM_ALIGNMENT_KD_NAME         "alignment_kd"
#define PARAM_ALIGNMENT_KD_DEFAULT      6.0
#define PARAM_ALIGNMENT_KI_NAME         "alignment_ki"
#define PARAM_ALIGNMENT_KI_DEFAULT      0.0

using namespace s8;
using namespace s8::wall_follower_controller_node;

#define IR_INVALID_VALUE ir_sensors_node::TRESHOLD_VALUE

class WallFollower : public Node {
    ros::Subscriber ir_distances_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_wall_follower_controller::FollowWallAction> follow_wall_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    double kp_near;
    double kp_far;
    double distance;
    double i_threshold;
    double i_limit;
    double ir_threshold;
    double following_kp;
    double following_kd;
    double following_ki;
    double alignment_kp;
    double alignment_kd;
    double alignment_ki;

    double right_prev_diff;
    double left_prev_diff;
    double sum_errors;

    double left_front;
    double left_back;
    double right_front;
    double right_back;

    double v;
    double w;
    double linear_speed;

    WallToFollow wall_to_follow;
    bool following;
    bool preempted;

    bool aligning;
    int alignment_streak;

public:
    WallFollower() : v(0.0), w(0.0), alignment_streak(0), aligning(false), following(false), preempted(false), left_front(IR_INVALID_VALUE), left_back(IR_INVALID_VALUE), right_front(IR_INVALID_VALUE), right_back(IR_INVALID_VALUE), right_prev_diff(0.0), left_prev_diff(0.0), sum_errors(0.0), stop_action(ACTION_STOP, true), follow_wall_action(nh, ACTION_FOLLOW_WALL, boost::bind(&WallFollower::action_execute_follow_wall_callback, this, _1), false) {
        init_params();
        print_params();

        ir_distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1, &WallFollower::ir_distances_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);
        
        follow_wall_action.registerPreemptCallback(boost::bind(&WallFollower::follow_wall_cancel_callback, this));

        follow_wall_action.start();

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

    void update() {
        if(!following) {
            return;
        }

        double back = wall_to_follow == WallToFollow::LEFT ? left_back : right_back;
        double front = wall_to_follow == WallToFollow::LEFT ? left_front : right_front;
        double prev_diff = wall_to_follow == WallToFollow::LEFT ? left_prev_diff : right_prev_diff;

        if(is_ir_valid_value(back) && is_ir_valid_value(front)) {
            if(aligning) {
                if(is_aligned(back, front)) {
                    if(alignment_streak == 1) {
                        ROS_INFO("Seconds alignment check");
                        aligning = false;
                        ROS_INFO("Alignment complete!");
                        v = 0.0;
                        w = 0.0;
                    } else {
                        ROS_INFO("First alignment check");
                        stop();
                        alignment_streak++;
                    }
                } else {
                    alignment_streak = 0;
                    ROS_INFO("Aligning %s wall... back: %.3lf, front: %.3lf", to_string(wall_to_follow).c_str(), back, front);
                    controller(back, front, prev_diff, (int)wall_to_follow, 0.0, alignment_kp, alignment_kd, alignment_ki, false);
                }
            }

            if(!aligning) {
                ROS_INFO("Following %s wall... back: %.2lf, front: %.2lf", to_string(wall_to_follow).c_str(), back, front);
                controller(back, front, prev_diff, (int)wall_to_follow, linear_speed, following_kp, following_kd, following_ki);
            }
        } else {
            ROS_INFO("Stopped following wall due to invalid ir sensor values. back: %lf, front: %lf", back, front);
            following = false;
            return;
        }

        publish();
    }

    void stop() {
        ROS_INFO("Stopping...");
        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            ROS_INFO("Stop action finished. %s", state.toString().c_str());
        } else {
            ROS_WARN("Stop action timed out.");
        }
    }
    
private:
    bool is_aligned(double back, double front) {
	ROS_INFO("alignment: back: %lf front: %lf diff: %lf", std::abs(back), std::abs(front), std::abs(std::abs(back) - std::abs(front)));
        return std::abs(std::abs(back) - std::abs(front)) <= 0.01;
    }

    void follow_wall_cancel_callback() {
        stop_following();

        ROS_INFO("Cancelling wall following action...");
    }

    void stop_following() {
        aligning = false;
        following = false;
        preempted = true;
        v = 0.0;
        w = 0.0;
    }

    void action_execute_follow_wall_callback(const s8_wall_follower_controller::FollowWallGoalConstPtr & goal) {
        wall_to_follow = WallToFollow(goal->wall_to_follow);

        const int timeout = 30; // 30 seconds.
        const int rate_hz = 10;

        ros::Rate rate(rate_hz);

        int ticks = 0;

        ROS_INFO("Wall following action started. Wall to follow: %s", to_string(wall_to_follow).c_str());

        preempted = false;
        following = true;
        aligning = true;
        alignment_streak = 0;
        while(ros::ok() && following && ticks <= timeout * rate_hz) {
            rate.sleep();
            ticks++;
        }

        if(!ros::ok()) {
            ROS_INFO("Ros not OK");
            return;
        }

        if(ticks >= timeout * rate_hz) {
            ROS_WARN("Wall following action timed out.");
            stop_following();
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = FollowWallFinishedReason::TIMEOUT;
            follow_wall_action.setAborted(follow_wall_action_result);
        } else if(preempted) {
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = FollowWallFinishedReason::PREEMPTED;
            follow_wall_action.setPreempted(follow_wall_action_result);        
        } else if(!preempted) {
            stop_following();
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = FollowWallFinishedReason::OUT_OF_RANGE;
            follow_wall_action.setSucceeded(follow_wall_action_result);
        }
    }

    void controller(double back, double front, double & prev_diff, double away_direction, double v_speed, double kp, double kd, double ki, bool do_distance = true) {
        double towards_direction = -away_direction;
       
        double diff = away_direction * (front - back);
        double average = (back + front) / 2;

        w = 0.0;
        p_controller(w, diff, kp);
        d_controller(w, diff, prev_diff, kd);

        // Maybe should change kp_near and kp_far not to be used when aligning?Z
        if(do_distance) {
            double distance_diff = average - distance;

            if(distance_diff < 0) {
                // if too close to the wall, turn away fast.
                w += -away_direction * kp_near * distance_diff;
            } else {
                //if too far away to the wall, turn towards slowly
                double w_temp = -towards_direction * kp_far * distance_diff;

                double towards_treshold = towards_direction * 0.2;

                if(std::abs((double)w_temp) > std::abs((double)towards_treshold)) {
                    w_temp = towards_treshold;
                }

                w -= w_temp;
            }

            if(std::abs(diff) < i_threshold) {
                i_controller(w, distance_diff, sum_errors, ki);
            } else {
                sum_errors = 0;
            }

            ROS_INFO("distance: %lf diff: %lf, added w: %lf", average, distance_diff, -away_direction * kp_near * distance_diff);
        }

        v = v_speed;

        ROS_INFO("Controller back: %.2lf, front: %.2lf, diff %lf, prev_diff: %.2lf, w: %.2lf", back, front, diff, prev_diff, w);
    }

    void p_controller(double & w, double diff, double kp) {
        w += -kp * diff;
    }

    void d_controller(double & w, double diff, double & prev_diff, double kd) {
        w += -kd * (diff - prev_diff) / 2;
        prev_diff = diff;
    }

    void i_controller(double & w, double diff, double & sum_errors, double ki){
        sum_errors += diff * (1.0 / HZ);
        w += -ki * sum_errors;
        
        // thresholding, tochange
        if (sum_errors > i_limit) {
            sum_errors = i_limit;
        } else if (sum_errors < -i_limit) {
            sum_errors = -i_limit;
        }
        
        ROS_INFO("error_diff: %lf, i: %lf", diff*(1.0/HZ), -ki*sum_errors);
    }

    void ir_distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        left_front = ir_distances->left_front;
        left_back = ir_distances->left_back;
        right_front = ir_distances->right_front;
        right_back = ir_distances->right_back;
    
        if(aligning) {
            ROS_INFO("Ir updated right_back: %lf right_front: %lf", right_back, right_front);
        }
    }

    bool is_ir_valid_value(double value) {
        return ir_sensors_node::is_valid_ir_value(value) && value < ir_threshold;
    }

    void publish() {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;

        twist_publisher.publish(twist);
    }

    void init_params() {
        add_param(PARAM_FOLLOWING_KP_NAME, following_kp, PARAM_FOLLOWING_KP_DEFAULT);
        add_param(PARAM_FOLLOWING_KD_NAME, following_kd, PARAM_FOLLOWING_KD_DEFAULT);
        add_param(PARAM_FOLLOWING_KI_NAME, following_ki, PARAM_FOLLOWING_KI_DEFAULT);
        add_param(PARAM_KP_NEAR_NAME, kp_near, PARAM_KP_NEAR_DEFAULT);
        add_param(PARAM_KP_FAR_NAME, kp_far, PARAM_KP_FAR_DEFAULT);
        add_param(PARAM_DISTANCE_NAME, distance, PARAM_DISTANCE_DEFAULT);
        add_param(PARAM_I_THRESHOLD_NAME, i_threshold, PARAM_I_THRESHOLD_DEFAULT);
        add_param(PARAM_I_LIMIT_NAME, i_limit, PARAM_I_LIMIT_DEFAULT);
        add_param(PARAM_LINEAR_SPEED_NAME, linear_speed, PARAM_LINEAR_SPEED_DEFAULT);
        add_param(PARAM_IR_THRESHOLD_NAME, ir_threshold, PARAM_IR_THRESHOLD_DEFAULT);
        add_param(PARAM_ALIGNMENT_KP_NAME, alignment_kp, PARAM_ALIGNMENT_KP_DEFAULT);
        add_param(PARAM_ALIGNMENT_KD_NAME, alignment_kd, PARAM_ALIGNMENT_KD_DEFAULT);
        add_param(PARAM_ALIGNMENT_KI_NAME, alignment_ki, PARAM_ALIGNMENT_KI_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    WallFollower wall_follower;

    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        wall_follower.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
