#ifndef __WALL_FOLLOWER_CONTROLLER_NODE_H
#define __WALL_FOLLOWER_CONTROLLER_NODE_H

#include <string>
#include <s8_motor_controller/motor_controller_node.h>
#include <s8_ir_sensors/ir_sensors_node.h>

namespace s8 {
    namespace wall_follower_controller_node {
        const std::string NODE_NAME =                       "s8_wall_follower_controller_node";

        const std::string TOPIC_IR_DISTANCES =              s8::ir_sensors_node::TOPIC_IR_DISTANCES;
        const std::string TOPIC_TWIST =                     s8::motor_controller_node::TOPIC_TWIST;
        const std::string ACTION_STOP =                     s8::motor_controller_node::ACTION_STOP;
        const std::string ACTION_FOLLOW_WALL =              "/s8/follow_wall";

        enum FollowWallFinishedReason {
            TIMEOUT,
            OUT_OF_RANGE,
            PREEMPTED
        };

        enum WallToFollow {
            LEFT = s8::motor_controller_node::RotationDirection::LEFT,
            RIGHT = s8::motor_controller_node::RotationDirection::RIGHT
        };

        std::string to_string(WallToFollow wall_to_follow) {
            switch(wall_to_follow) {
                case WallToFollow::LEFT: return "LEFT";
                case WallToFollow::RIGHT: return "RIGHT";
            }

            return "UNKNOWN";
        }
    }
}

#endif
