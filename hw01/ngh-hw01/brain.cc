
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

void faceGoal(Robot* robot) {
    float curr_angle = robot->pos_t;
    // Calculating the goal coords based on the robots curr position (0,0)
    float resp_goal_x = goal_x - robot->pos_x;
    float resp_goal_y = goal_y - robot->pos_y;

    // Get the angle the robot must change to move to target
    float angle_arc_tan = atan2(resp_goal_y, resp_goal_x); // In tanget arc
    float turn_amount = curr_angle - angle_arc_tan;

    cout << "arc tan: " << angle_arc_tan << " turn amount: " << turn_amount << "\n";

    robot->set_turn(turn_amount);
}

void
callback(Robot* robot)
{
    /*
    cout << endl;
    cout << "robot x =" << robot->pos_x << endl;
    cout << "robot y =" << robot->pos_y << endl;
    */
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    bool turn = false;

    for (LaserHit hit : robot->hits) {
        if (hit.range < 1.5) {
            if (hit.angle < 0.5 || hit.angle > (6.2 - 0.5)) {
                turn = true;
            }
        }
    }

    if (turn) {
        robot->set_vel(3.0);
        robot->set_turn(0.5);
    }
    else {
        robot->set_vel(5.0);
        faceGoal(robot);
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
