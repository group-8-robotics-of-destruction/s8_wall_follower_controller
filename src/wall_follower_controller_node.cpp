#include <cmath>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <s8_common_node/Node.h>

#include <geometry_msgs/Twist.h>
#include <s8_msgs/IRDistances.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>

#define NODE_NAME                       "s8_wall_follower_controller_node"
#define HZ                              10

#define TOPIC_TWIST                     "/s8/twist"
#define TOPIC_IR_DISTANCES              "/s8/ir_distances"
#define ACTION_STOP                     "/s8_motor_controller/stop"
#define ACTION_FOLLOW_WALL              "/s8/follow_wall"

#define PARAM_KP_NAME                   "kp"
#define PARAM_KP_DEFAULT                5.0
#define PARAM_KD_NAME                   "kd"
#define PARAM_KD_DEFAULT                6.0
#define PARAM_KP_NEAR_NAME              "kp_near"
#define PARAM_KP_NEAR_DEFAULT           0.9
#define PARAM_KI_NAME                   "ki"
#define PARAM_KI_DEFAULT                0.4
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

#define IR_INVALID_VALUE                -1.0
#define REASON_TIMEOUT                  1
#define REASON_OUT_OF_RANGE             1 << 1
#define REASON_PREEMPTED                1 << 2

class WallFollower : public s8::Node {
public:
    enum FollowWall {
        LEFT = -1,
        RIGHT = 1
    };

private:
    ros::Subscriber ir_distances_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_wall_follower_controller::FollowWallAction> follow_wall_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    double kp_near;
    double kp_far;
    double kp;
    double kd;
    double ki;
    double distance;
    double i_threshold;
    double i_limit;

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

    FollowWall wall_to_follow;
    bool following;
    bool preempted;

    bool aligning;

public:
    WallFollower() : v(0.0), w(0.0), aligning(false), following(false), preempted(false), left_front(IR_INVALID_VALUE), left_back(IR_INVALID_VALUE), right_front(IR_INVALID_VALUE), right_back(IR_INVALID_VALUE), right_prev_diff(0.0), left_prev_diff(0.0), sum_errors(0.0), stop_action(ACTION_STOP, true), follow_wall_action(nh, ACTION_FOLLOW_WALL, boost::bind(&WallFollower::action_execute_follow_wall_callback, this, _1), false) {
        init_params();
        print_params();

        ir_distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1000, &WallFollower::ir_distances_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1000);
        
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

        double back = wall_to_follow == WallFollower::LEFT ? left_back : right_back;
        double front = wall_to_follow == WallFollower::LEFT ? left_front : right_front;
        double prev_diff = wall_to_follow == WallFollower::LEFT ? left_prev_diff : right_prev_diff;

        if(is_ir_valid_value(back) && is_ir_valid_value(front)) {
            if(aligning) {
                if(is_aligned(back, front)) {
                    aligning = false;
                } else {
                    ROS_INFO("Aligning %s wall... back: %.2lf, front: %.2lf", wall_to_follow == WallFollower::LEFT ? "left" : "right", back, front);
                    controller(back, front, prev_diff, (int)wall_to_follow, 0.0, false);
                }
            }

            if(!aligning) {
                ROS_INFO("Following %s wall... back: %.2lf, front: %.2lf", wall_to_follow == WallFollower::LEFT ? "left" : "right", back, front);
                controller(back, front, prev_diff, (int)wall_to_follow, linear_speed); 
            }
        } else {
            following = false;
            return;
        }

        publish();
    }

    void stop() {
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
        return std::abs(std::abs(back) - std::abs(front)) < 0.01;
    }

    void follow_wall_cancel_callback() {
        stop_following();

        s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
        follow_wall_action_result.reason = REASON_PREEMPTED;
        follow_wall_action.setPreempted(follow_wall_action_result);

        ROS_INFO("Cancelling wall following action...");
    }

    void stop_following() {
        following = false;
        preempted = true;
        v = 0.0;
        w = 0.0;
    }

    void action_execute_follow_wall_callback(const s8_wall_follower_controller::FollowWallGoalConstPtr & goal) {
        wall_to_follow = FollowWall(goal->wall_to_follow);

        const int timeout = 30; // 30 seconds.
        const int rate_hz = 10;

        ros::Rate rate(rate_hz);

        int ticks = 0;

        ROS_INFO("Wall following action started. Wall to follow: %s", wall_to_follow == WallFollower::LEFT ? "left" : "right");

        preempted = false;
        following = true;
        aligning = true;
        while(following && ticks <= timeout * rate_hz) {
            rate.sleep();
            ticks++;
        }

        if(ticks >= timeout * rate_hz) {
            ROS_WARN("Wall following action timed out.");
            stop_following();
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = REASON_TIMEOUT;
            follow_wall_action.setAborted(follow_wall_action_result);
        } else if(!preempted) {
            stop_following();
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = REASON_OUT_OF_RANGE;
            follow_wall_action.setSucceeded(follow_wall_action_result);
        }
    }

    void controller(double back, double front, double & prev_diff, double away_direction, double v_speed, bool do_distance = true) {
        double diff = front - back;
        double average = (back + front) / 2;

        w = 0.0;
        p_controller(w, diff);
        d_controller(w, diff, prev_diff);

        if(do_distance) {
            double towards_direction = -away_direction;
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
                i_controller(w, distance_diff, sum_errors);
            } else {
                sum_errors = 0;
            }
        }

        v = v_speed;

        ROS_INFO("Controller back: %.2lf, front: %.2lf, diff %lf, w: %.2lf", right_back, right_front, diff, w);
    }

    void p_controller(double & w, double diff) {
        w += -kp * diff;
    }

    void d_controller(double & w, double diff, double & prev_diff) {
        w += -kd * (diff - prev_diff) / 2;
        prev_diff = diff;
    }

    void i_controller(double & w, double diff, double & sum_errors){
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
    }

    bool is_ir_valid_value(double value) {
        return value > 0.0;
    }

    void publish() {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;

        twist_publisher.publish(twist);
    }

    void init_params() {
        add_param(PARAM_KP_NAME, kp, PARAM_KP_DEFAULT);
        add_param(PARAM_KD_NAME, kd, PARAM_KD_DEFAULT);
        add_param(PARAM_KP_NEAR_NAME, kp_near, PARAM_KP_NEAR_DEFAULT);
        add_param(PARAM_KP_FAR_NAME, kp_far, PARAM_KP_FAR_DEFAULT);
        add_param(PARAM_KI_NAME, ki, PARAM_KI_DEFAULT);
        add_param(PARAM_DISTANCE_NAME, distance, PARAM_DISTANCE_DEFAULT);
        add_param(PARAM_I_THRESHOLD_NAME, i_threshold, PARAM_I_THRESHOLD_DEFAULT);
        add_param(PARAM_I_LIMIT_NAME, i_limit, PARAM_I_LIMIT_DEFAULT);
        add_param(PARAM_LINEAR_SPEED_NAME, linear_speed, PARAM_LINEAR_SPEED_DEFAULT);
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
