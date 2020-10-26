
#include <iostream>
#include <thread>
#include <math.h>
#include <cmath>

#include "robot.hh"
extern "C" {
    #include "gfx.h"
}
#include "viz.hh"
#include "cell.hh"

using std::cout;
using std::endl;
using std::vector;

static int windowX = 600;
static int windowY = 600;

static int MAZE_SIZE = 60;
static int FACTOR_PER_METER = 4;
static int grid_size = 300;
static int occupiedScore = 1;
static int freeScore = 1;

int turn_tick_count = 0;
int tick_count = 0;
int tick_global_count = 0;

Cell map_2d[600][600];

std::vector<Cell> bresenhamLineAlgo(int x1, int y1, int x2, int y2) {
    std::vector<Cell> result;

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1) * -1;

    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;

    int error = dx + dy;

    int counter = 0;

    while (1) {
        // Add the current dx and dy into cell list
        Cell currCell(x1, y1);
        result.push_back(currCell);

        // Move to next cell
        if (x1 == x2 && y1 == y2) {
            break;
        }

        int tempError = 2 * error;
        if (tempError > dy) {
            error = error + dy;
            x1 = x1 + sx;
        }

        if (tempError < dx) {
            error = error + dx;
            y1 = y1 + sy;
        }
    }

    return result;
}

void initilize_2dMap() {
    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            Cell currCell = Cell();
            map_2d[i][j] = currCell;
        }
    }
}

void drawXY2(int x, int y, bool freeColor) {
    int xx = x * 2;
    int yy = y * 2;

    if (freeColor) {
        gfx_color(0, 255, 0);
    } else {
        gfx_color(255, 0, 0);
    }

    gfx_point(x, y);
}


void draw2DMap() {
    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            Cell currCell = map_2d[i][j];

            if (currCell.hits < -2) {
                drawXY2(i, j, true);
            }

            if (currCell.hits > 1.0) {
                drawXY2(i, j, false);
                continue;
            }
        }
    } 
}

void createWindow() {
    gfx_open(windowX, windowY, "Map");

    char c;

    gfx_clear_color(255, 255, 255);
    gfx_clear();

    gfx_color(0, 0, 0);

    while(1) {
        c = gfx_wait();

        if (c == 'm') {
            draw2DMap();
        }

        if (c =='q') {
            break;
        }
    }
}

void trackPath(Robot* robot) {
    int x_in_2dMap = ceil(robot->pos_x * 4);
    int y_in_2dMap = ceil(robot->pos_y * 4);

    int xx = (grid_size / 2) + x_in_2dMap;
    int yy = (grid_size / 2) - y_in_2dMap;

    Cell currCell = map_2d[xx][yy];
    currCell.hits += 1;

    drawXY2(xx, yy, false);

    cout << "x,y" << robot->pos_x << "," << robot->pos_y << endl;
    cout << "xx,yy,c: " << xx << "," << yy << "," << currCell.hits << endl;
}

void printCellList(std::vector<Cell> givenCells) {
    cout << "FIRST CELL: " << givenCells[0].x << "," << givenCells[0].y << endl;
    for (int i = 1; i < givenCells.size() - 1; i++) {
        cout << givenCells[i].x << "," << givenCells[i].y << endl;
    }

    cout << "LAST CELL: " << givenCells[givenCells.size() - 1].x << "," << givenCells[givenCells.size() - 1].y << endl;
}

void
callback(Robot* robot)
{
    tick_count += 1;

    if (tick_count > 20) {
        tick_count = 0;
        draw2DMap();
    }

    cout << "\n===" << endl;

    cout << "tick: " << tick_global_count << endl;
    tick_global_count++;

    for (auto hit : robot->ranges) {
        float hit_angle, robot_angle, dx, dy, detectedX, detectedY;

        int NO_HIT_RANGE = 3;

        if (hit.range < 3) {
            NO_HIT_RANGE = hit.range;
        }

        // Right facing
        if (-M_PI <= robot->pos_t && robot->pos_t <= 0) {
            robot_angle = - abs(robot->pos_t);
        }

        // Left facing
        if (0 <= robot->pos_t && robot->pos_t <= M_PI) {
            robot_angle = abs(robot->pos_t);
        }

        hit_angle = hit.angle;

        if (hit_angle > M_PI || hit_angle < -M_PI || robot_angle > M_PI || robot_angle < -M_PI) {
            break;
        }

        dx = NO_HIT_RANGE * cos(hit_angle + robot_angle);
        dy = NO_HIT_RANGE * sin(hit_angle + robot_angle);

        detectedX = robot->pos_x + dx;
        detectedY = robot->pos_y + dy;

        int x_in_2dMap = (grid_size / 2) + ceil(detectedX * 4);
        int y_in_2dMap = (grid_size / 2) - ceil(detectedY * 4);

        int robo_x_in_2dMap = (grid_size / 2) + ceil(robot->pos_x * 4);
        int robo_y_in_2dMap = (grid_size / 2) - ceil(robot->pos_y * 4);

        std::vector<Cell> inBetweenCells = bresenhamLineAlgo(robo_x_in_2dMap, robo_y_in_2dMap, x_in_2dMap, y_in_2dMap);

        // Update all cells between current pos to final cell - 1 to be free
        for (int i = 0; i < inBetweenCells.size() - 1; i++) {
            Cell currCell = inBetweenCells[i];

            if (currCell.x < 0 || currCell.y < 0) {
                continue;
            }
            map_2d[currCell.x][currCell.y].hits -= 0.33;
        }


        if (hit.range < 3) {

            // Updating the final cell that is not free
            Cell finalCell = inBetweenCells[inBetweenCells.size() - 1];
            if (finalCell.x >= 0 && finalCell.y >= 0) {
                map_2d[finalCell.x][finalCell.y].hits += 0.9;
            }

            float bearing_from_heading = hit_angle + robot_angle;

            cout << "A: " << hit.angle << " A,Bearing: " << bearing_from_heading << " hr: " << hit.range
            << " x,y,t: " << robo_x_in_2dMap << "," << robo_y_in_2dMap << "," << robot->pos_t
            << " dx,dy: " << x_in_2dMap << ","  << y_in_2dMap << endl;
        }

        // cout << hit.range << "@" << hit.angle << endl;
    }

    if (robot->ranges.size() < 5) {
        return;
    }

    float rgt = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float lft = clamp(0.0, robot->ranges[4].range, 2.0);

    float spd = 1;
    float trn = 0;

    // Turn logic
    if (fwd < 1.3 || turn_tick_count > 0) {
        // Front sensor has been hit therefore turn to left
        spd = 0;
        trn = 2;
        turn_tick_count++;

        if (turn_tick_count == 60) {
            turn_tick_count = 0;
        }

        robot->set_vel(spd + trn, spd - trn);
        return;
    }

    // Wall follow logic
    // If lft_90 is not within this range adjust robot to kiss wall
    if (1 > lft || lft > 1.5) {
        if (lft < 1) { // Too close to wall turn left
            spd = 1;
            trn = 2;
        }

        if (lft > 1.5) {
            spd = 1;
            trn = -2;
        }
    } else {
        spd = 2;
        trn = 0;
    }


    robot->set_vel(spd + trn, spd - trn);

    // cout << "spd,trn = " << spd << "," << trn << endl;
    // cout << "lft,fwd,rgt = "
    //     << lft << ","
    //     << fwd << ","
    //     << rgt << endl;

    // cout << "x,y,t = "
    //      << robot->pos_x << ","
    //      << robot->pos_y << ","
    //      << robot->pos_t << endl;
    // robot->set_vel(robot->pos_t, -robot->pos_t);
    
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;

    initilize_2dMap();
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);

    createWindow();

    return 0;

    // return viz_run(argc, argv);
}
