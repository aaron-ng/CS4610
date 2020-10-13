
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;


static int DOORWAYS_PER_HALLWAY = 2;

bool face_east = false;
bool face_west = false;

// Maze state
int hallway_count = 1;

// Door Bools
int doorways_found = 0;
int backwards_doorways_found = 0;
int ticks_without_obstacle = 0;
int ticks_to_move_south = 0;
int ticks_move_forward = 0;
bool going_through_door = false;
bool middle_of_door_way = false;
bool middle_of_backwards_door_way = false;
bool past_door = false;

void printState(Robot* robot) {
    cout << "Middle of doorway: " << middle_of_door_way << endl;
    cout << "Ticks to move south: " << ticks_to_move_south << " ticks without obs: " << ticks_without_obstacle << endl;
    cout << "Current hallway: " << hallway_count << endl;
    cout << " Doorways found: " << doorways_found << " Backwards doorways: " << backwards_doorways_found << endl;
    cout << "Robot Pos:" << (robot->pos_t) << endl;
    cout << "Robot range: " << (robot->range) << endl;
}

bool isNorth(Robot* robot) {
    return -0.05 < robot->pos_t && robot->pos_t < 0.05;
}

bool isWest(Robot* robot) {
    return 1.55 < robot->pos_t && robot->pos_t < 1.59;
}

bool isBroadlyWest(Robot* robot) {
    return 1.45 < robot->pos_t && robot->pos_t < 1.65;
}

bool isSouth(Robot* robot) {
    return 3.10 < robot->pos_t || robot->pos_t < -3.13;
}

bool isEast(Robot* robot) {
    return -1.58 < robot->pos_t && robot->pos_t < -1.56;
}

bool isBroadlyEast(Robot* robot) {
    return -1.64 < robot->pos_t && robot->pos_t < -1.50;
}

void go_face_east(Robot* robot) {
    robot->set_vel(2.0, -2.0);
    if (isEast(robot)) {
        face_east = false;
    }

    return;
}

void go_face_west(Robot* robot) {
    robot->set_vel(-2.0, 2.0);
    if (isWest(robot)) {
        face_west = false;
    }

    return;
}

void finish_door_mnv(Robot* robot) {
    doorways_found = 0;
    ticks_without_obstacle = 0;
    ticks_to_move_south = 0;
    ticks_move_forward = 0;
    hallway_count++;
    going_through_door = false;
    past_door = false;
}

// We are at the final hallway. need to go back. We want to take the first avaible doorway
// The first available doorway guarantees it is not the doorway we came from
void lastHallwayLogic(Robot* robot) {
    if (backwards_doorways_found >= 1) {
            // Turn to forwards
        if (!isSouth(robot) && !past_door) {
            robot->set_vel(1.5, -1.5);
            return;
        }

        // if facing _forwards
        if (isSouth(robot)) {
            past_door = true;
            robot->set_vel(3.0, 3.0);
        }

        if (isBroadlyWest(robot) && robot->range > 1.5) {
            finish_door_mnv(robot);
        }

        if (isBroadlyEast(robot) && robot->range > 1.3) {
            finish_door_mnv(robot);
        }

        // Turn left until we find available path
        if (robot->range < 1.2 || ticks_to_move_south > 0) {
            robot->set_vel(-1, 1);
        }

        return;
    }
} 


// Robot position:
// 0 = North
// 1.57 = East (Pi / 2)
// 3.14 = South (pi)
// -1.57 = West (-Pi / 2)

void
callback(Robot* robot)
{

    printState(robot);

    if (robot->at_goal()) {
        robot->done();
        return;
    }

    if (face_east) {
        go_face_east(robot);
    }

    if (face_west) {
        go_face_west(robot);
    }

    // We want to be in the middle to reach goal
    // Moving in a straight line logic
    if (hallway_count == 11) {
        // If there is a obstacle turn left
        if (robot->range < 1.5) {
            robot->set_vel(-1, 1);
            return;
        }

        // If there is no obstacle go forwards
        robot->set_vel(4.0, 4.0);
    }

    if (hallway_count == 12) {
        lastHallwayLogic(robot);
        return;
    }

    // If we found all the doorways, then proceed to move forward
    if (doorways_found >= 1) {

        // Middle of the door
        if (ticks_without_obstacle < 4) {
            ticks_without_obstacle++;
            return;
        }

        // Turn to face north
        if (!isNorth(robot) && !past_door) {
            robot->set_vel(1.5, -1.5);
            cout << "Turn robot to goal, pos: " << robot->pos_t << endl;
            return;
        }

        // if facing north, then move forwards
        if (isNorth(robot)) {
            past_door = true;
            ticks_move_forward++;

            robot->set_vel(3.0, 3.0);

            cout << "Moving North Tick: " << ticks_move_forward << endl;

            if (ticks_move_forward > 100) {
                hallway_count++;
                ticks_move_forward = 0;
            }
        }

        if (isBroadlyWest(robot) && robot->range > 1.5) {
            finish_door_mnv(robot);
        }

        if (isBroadlyEast(robot) && robot->range > 1.3) {
            finish_door_mnv(robot);
        }

        if (isSouth(robot) && ticks_to_move_south < 35) {
            robot->set_vel(2, 2);
            ticks_to_move_south++;
            return;
        }

        // Turn left until we find available path
        if (robot->range < 1.2 || ticks_to_move_south > 0) {
            robot->set_vel(-1, 1);
        }

        return;
    }

    // Found forward doorway. Turn into it and keep going forwards
    if (isBroadlyWest(robot) && robot->range > 900) {
        middle_of_door_way = true;
    }

    // Found backwards doorway. Turn into it and keep going forwards
    if (isBroadlyEast(robot) && robot->range > 900) {
        middle_of_backwards_door_way = true;
    }

    if (middle_of_backwards_door_way and robot->range < 2) {
        middle_of_backwards_door_way = false;
        backwards_doorways_found++;
    }

    if (middle_of_door_way and robot->range < 2) {
        middle_of_door_way = false;
        doorways_found++;
    }

    // If there is a obstacle turn left
    if (robot->range < 1.1) {
        robot->set_vel(-1, 1);
        return;
    }

    // If there is no obstacle go forwards
    robot->set_vel(4.0, 4.0);
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
