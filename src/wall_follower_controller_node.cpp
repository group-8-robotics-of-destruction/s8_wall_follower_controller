#include <ros/ros.h>
#include <cmath>

#include <s8_common_node/Node.h>
#include <s8_utils/math.h>
#include <s8_wall_follower_controller/wall_follower_controller_node.h>
#include <s8_pid/PIDController.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Twist.h>
#include <s8_msgs/IRDistances.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>

#define HZ                              25

#define PARAM_FOLLOWING_KP_NAME         "following_kp"
#define PARAM_FOLLOWING_KP_DEFAULT      10.0
#define PARAM_FOLLOWING_KD_NAME         "following_kd"
#define PARAM_FOLLOWING_KD_DEFAULT      4.0
#define PARAM_FOLLOWING_KI_NAME         "following_ki"
#define PARAM_FOLLOWING_KI_DEFAULT      0.0
#define PARAM_DISTANCE_KI_NAME          "distance_ki"
#define PARAM_DISTANCE_KI_DEFAULT       0.0
#define PARAM_DISTANCE_KP_NAME          "distance_kp"
#define PARAM_DISTANCE_KP_DEFAULT       10.0
#define PARAM_DISTANCE_KD_NAME          "distance_kd"
#define PARAM_DISTANCE_KD_DEFAULT       4.0
#define PARAM_I_LIMIT_NAME              "i_limit"
#define PARAM_I_LIMIT_DEFAULT           0.5
#define PARAM_DISTANCE_NAME             "distance"
#define PARAM_DISTANCE_DEFAULT          0.09
#define PARAM_LINEAR_SPEED_NAME         "linear_speed"
#define PARAM_LINEAR_SPEED_DEFAULT      0.2
#define PARAM_IR_THRESHOLD_NAME         "ir_threshold"
#define PARAM_IR_THRESHOLD_DEFAULT      0.2
#define PARAM_ALIGNMENT_KP_NAME         "alignment_kp"
#define PARAM_ALIGNMENT_KP_DEFAULT      0.1
#define PARAM_ALIGNMENT_KD_NAME         "alignment_kd"
#define PARAM_ALIGNMENT_KD_DEFAULT      0.3
#define PARAM_ALIGNMENT_KI_NAME         "alignment_ki"
#define PARAM_ALIGNMENT_KI_DEFAULT      0.0
#define PARAM_ALIGNMENT_SPEED_NAME      "alignment_speed"
#define PARAM_ALIGNMENT_SPEED_DEFAULT   0.5

using namespace s8;
using namespace s8::wall_follower_controller_node;
using namespace s8::pid;
using namespace s8::utils::math;

#define IR_INVALID_VALUE ir_sensors_node::TRESHOLD_VALUE

class WallFollower : public Node {
    ros::Subscriber ir_distances_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_wall_follower_controller::FollowWallAction> follow_wall_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    double distance;
    double ir_threshold;
    PIDController follow_pid;
    PIDController align_pid;
    PIDController distance_pid;

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
    int alignment_angle;
    double alignment_speed;

public:
    WallFollower(int hz) : distance_pid (hz), follow_pid(hz), align_pid(hz), v(0.0), w(0.0), alignment_streak(0), alignment_angle(0), aligning(false), following(false), preempted(false), left_front(IR_INVALID_VALUE), left_back(IR_INVALID_VALUE), right_front(IR_INVALID_VALUE), right_back(IR_INVALID_VALUE), stop_action(ACTION_STOP, true), follow_wall_action(nh, ACTION_FOLLOW_WALL, boost::bind(&WallFollower::action_execute_follow_wall_callback, this, _1), false) {
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

        //TODO: Use sign instead in utils.
        auto get_alignment_angle = [&back, &front]() {
            return back - front >= 0 ? 1 : -1;
        };

        if(is_ir_valid_value(back) && is_ir_valid_value(front)) {
            if(aligning) {
                if(alignment_angle != 0) {
                    if(alignment_angle != get_alignment_angle()) {
                        ROS_INFO("Alignment angle sign changed.");
                        align_pid.reset();
                    }
                }

                alignment_angle = get_alignment_angle();

                if(is_aligned(back, front)) {
                    if(alignment_streak == 1) {
                        ROS_INFO("Seconds alignment check");
                        aligning = false;
                        ROS_INFO("Alignment complete!");
                        v = 0.0;
                        w = 0.0;
                    } else {
                        ROS_INFO("First alignment check");
                        alignment_streak++;
                        stop();
                        w = 0.0;
                    }
                } else {
                    alignment_streak = 0;
                    align_controller(back, front, wall_to_follow);
                }
            }

            if(!aligning) {
                follow_controller(back, front, wall_to_follow);
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
        return std::abs(std::abs(back) - std::abs(front)) <= 0.02;
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
        preempted = false;
        following = true;
        aligning = true;
        
        wall_to_follow = WallToFollow(goal->wall_to_follow);

        const int timeout = 30; // 30 seconds.
        const int rate_hz = 10;

        ros::Rate rate(rate_hz);

        int ticks = 0;

        ROS_INFO("Wall following action started. Wall to follow: %s", to_string(wall_to_follow).c_str());

        follow_pid.reset();
        distance_pid.reset();
        align_pid.reset();
        alignment_streak = 0;
        alignment_angle = 0;
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
            ROS_INFO("PREEMPTED: Cancelled wall following");
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = FollowWallFinishedReason::PREEMPTED;
            follow_wall_action.setPreempted(follow_wall_action_result);        
        } else if(!preempted) {
            ROS_INFO("SUCCEEDED: Wall following succeeded");
            stop_following();
            s8_wall_follower_controller::FollowWallResult follow_wall_action_result;
            follow_wall_action_result.reason = FollowWallFinishedReason::OUT_OF_RANGE;
            follow_wall_action.setSucceeded(follow_wall_action_result);
        }
    }

    double get_controller_diff(double back, double front, int away_direction) {
        return away_direction * (back - front);
    }

    void follow_controller(double back, double front, int towards_direction, bool do_distance = true) {
        int away_direction = -towards_direction;

        double diff = get_controller_diff(back, front, away_direction);
        double average = (back + front) / 2;

        w = 0.0;

        follow_pid.update(w, diff);

        v = linear_speed;

        // Maybe should change kp_near and kp_far not to be used when aligning?Z
        if(do_distance) {
            double distance_diff = distance - average;
            distance_controller(distance_diff, towards_direction);
        }

        ROS_INFO("Follow controller back: %.2lf, front: %.2lf, diff %lf, prev_diff: %.2lf, w: %.2lf", back, front, diff, follow_pid.get_prev_error(), w);
    }

    void distance_controller(double distance_diff, int towards_direction){
        int away_direction = -towards_direction;
        double diff = distance_diff * away_direction;
        // Add back the kp_far and near difference perhaps?
        if (distance_diff < 0) {
            int old_kp = distance_pid.kp;
            distance_pid.kp /= 5;
            distance_pid.update(w,diff);
            distance_pid.kp = old_kp;
        }
        else {
            distance_pid.update(w, diff);
        }
        ROS_INFO("distance difference: %lf",  diff);
    }

    void align_controller(double back, double front, int towards_direction) {
        double away_direction = -towards_direction;
        double diff = get_controller_diff(back, front, away_direction);
        w = sign(diff) * alignment_speed;
        //align_pid.update(w, diff);
        v = 0.0;
        ROS_INFO("Alignment controller back: %.2lf, front: %.2lf, diff %lf, prev_diff: %.2lf, w: %.2lf, towards: %d", back, front, diff, align_pid.get_prev_error(), w, towards_direction);
    }

    void ir_distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        left_front = ir_distances->left_front;
        left_back = ir_distances->left_back;
        right_front = ir_distances->right_front;
        right_back = ir_distances->right_back;
    }

    bool is_ir_valid_value(double value) {
        return ir_sensors_node::is_valid_ir_value(value) && value < ir_threshold;
    }

    void publish() {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        // Check for too high angular speed
        if (w > 1.5 || w < -1.5)  {
            ROS_WARN("Possibly bad IR sensor value");
            w = 0;  
        }

        twist.angular.z = w;
        twist_publisher.publish(twist);
    }

    void init_params() {
        add_param(PARAM_FOLLOWING_KP_NAME, follow_pid.kp, PARAM_FOLLOWING_KP_DEFAULT);
        add_param(PARAM_FOLLOWING_KD_NAME, follow_pid.kd, PARAM_FOLLOWING_KD_DEFAULT);
        add_param(PARAM_FOLLOWING_KI_NAME, follow_pid.ki, PARAM_FOLLOWING_KI_DEFAULT);
        add_param(PARAM_DISTANCE_KI_NAME, distance_pid.ki, PARAM_DISTANCE_KI_DEFAULT);
        add_param(PARAM_DISTANCE_KP_NAME, distance_pid.kp, PARAM_DISTANCE_KP_DEFAULT);
        add_param(PARAM_DISTANCE_KD_NAME, distance_pid.kd, PARAM_DISTANCE_KD_DEFAULT);
        add_param(PARAM_I_LIMIT_NAME, distance_pid.i_limit, PARAM_I_LIMIT_DEFAULT);
        add_param(PARAM_DISTANCE_NAME, distance, PARAM_DISTANCE_DEFAULT);
        add_param(PARAM_LINEAR_SPEED_NAME, linear_speed, PARAM_LINEAR_SPEED_DEFAULT);
        add_param(PARAM_IR_THRESHOLD_NAME, ir_threshold, PARAM_IR_THRESHOLD_DEFAULT);
        add_param(PARAM_ALIGNMENT_KP_NAME, align_pid.kp, PARAM_ALIGNMENT_KP_DEFAULT);
        add_param(PARAM_ALIGNMENT_KD_NAME, align_pid.kd, PARAM_ALIGNMENT_KD_DEFAULT);
        add_param(PARAM_ALIGNMENT_KI_NAME, align_pid.ki, PARAM_ALIGNMENT_KI_DEFAULT);
        add_param(PARAM_ALIGNMENT_SPEED_NAME, alignment_speed, PARAM_ALIGNMENT_SPEED_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    WallFollower wall_follower(HZ);

    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        wall_follower.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
