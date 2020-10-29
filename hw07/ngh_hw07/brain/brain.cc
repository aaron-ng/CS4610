
#include <iostream>
#include <thread>
#include <math.h>
#include <cmath>
#include <list>

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
static float occupiedMin = 1.0;
static float freeMin = -2.0;

static int GOAL_X = (grid_size / 2) + ceil(20 * 4);
static int GOAL_Y = (grid_size / 2) - ceil(0 * 4);


int turn_tick_count = 0;
int tick_count = 0;
int tick_global_count = 0;
int tick_path_count = 0;
int tick_wallFollow_count = 0;
bool makingTurn = false;

Cell map_2d[300][300]; // MAIN MAP

// Testing A* Algo
// Cell map_2d[300][300];
// static int GOAL_X = 230;
// static int GOAL_Y = 150;
// static int grid_size = 300;

// Directions around a cell. Up/Down/Left/Right etc.
std::vector<Cell> direction = {
        Cell(0, 1), Cell(1, 0), Cell(0, -1), Cell(-1, 0), Cell(-1, -1), Cell(1, 1), Cell(-1, 1), Cell(1, -1)
    };

// The actual path that the robot should follow.
std::vector<Cell> path;
int currentPathIndex = 0;
bool pathMaze = false;
bool currentlyPathing = false;
bool initialPath = true;

void printCell(Cell givenCell) {

    cout << "Cell x,y,g,h: " 
    << givenCell.x << "," 
    << givenCell.y << ","
    << givenCell.Gscore << "," 
    << givenCell.Hscore << endl;

    Cell* parentCell = givenCell.parent;

    if (parentCell == nullptr) {
        cout << "NULLPTR PARENT" << endl;
    } else {
        cout << "Parent Cell x,y,g,h: " 
        << parentCell->x << "," 
        << parentCell->y << ","
        << parentCell->Gscore << "," 
        << parentCell->Hscore << endl;
    }
}

void printCellList2(std::list<Cell> givenCells) {
    std::list<Cell>::iterator it;

    cout << "BEGIN CELL LIST" << endl;

    for (it = givenCells.begin(); it != givenCells.end(); it++) {
        cout << "x,y,g,h: " << it->x << "," << it->y << "," <<  it->Gscore << "," << it->Hscore << endl;
    }

    cout << "END CELL LIST" << endl;
}

double getScore(Cell givenCell) {
    return givenCell.Hscore + givenCell.Gscore;
}

bool isOccupied(int x, int y) {
    return x < 0 || y < 0 || x >= grid_size || y >= grid_size || map_2d[x][y].hits > occupiedMin;
}

Cell* findCell2(std::list<Cell*> givenloc, int givX, int givY) {
    for (auto cell: givenloc) {
        if (cell->x == givX && cell->y == givY) {
            return cell;
        }
    }

    return nullptr;
}

double euclidean(int initialX, int initialY, int targetX, int targetY) {
    double dx = abs(initialX - targetX);
    double dy = abs(initialY - targetY);

    return 10 * sqrt(pow(dx, 2) + pow(dy, 2));
}


/**
 *  Heavily modified A star algo based on this github: https://github.com/daancode/a-star
 *  All that was kept was the math calculation and overarching idea.
 */
std::vector<Cell> aStarAlgo(int initialX, int initialY, int goalX, int goalY) {
    std::list<Cell*> openCell, closeCell;

    openCell.push_back(new Cell(initialX, initialY));

    Cell *current = nullptr;

    while(!openCell.empty()) {
        auto current_it = openCell.begin();
        current = *current_it;

        // Get the lowest score possible in openCell
        for (auto it = openCell.begin(); it != openCell.end(); it++) {
            auto cellTemp = *it;

            // MAY NEED TO CHANGE FROM < TO <= IN FUTURE
            if (getScore(*cellTemp) < getScore(*current)) {
                current = cellTemp;
                current_it = it;
            }
        }

        if (current->x == goalX && current->y == goalY) {
            break;
        }

        closeCell.push_back(current);
        openCell.erase(current_it);

        for (int i = 0; i < direction.size(); i++) {
            int newX = current->x + direction[i].x;
            int newY =  current->y + direction[i].y;

            // If the current cell is occupied or in closeOpen move on
            if (isOccupied(newX, newY) || findCell2(closeCell, newX, newY)) {
                continue;
            }

            double totalCost = current->Gscore + ((i < 4) ? 10 : 14);

            Cell *successor = findCell2(openCell, newX, newY);
            
            if (successor == nullptr) {
                successor = new Cell(newX, newY, current);
                successor->Gscore = totalCost;
                successor->Hscore = euclidean(successor->x, successor->y, goalX, goalY);
                openCell.push_back(successor);
            } else if (totalCost < successor->Gscore) {
                successor->parent = current;
                successor->Gscore = totalCost;
            }
        }
    }

    std::vector<Cell> result;
    while (current != nullptr) {
        Cell currCell = Cell(current->x, current->y);
        result.push_back(currCell);

        if (current->x == initialX && current->y == initialY) {
            break;
        }

        current = current->parent;
    }

    // cout << "ENDING A* Algo. PATH LENGTH: " << result.size() << endl;

    openCell.clear();
    closeCell.clear();

    return result;
}

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
            Cell currCell = Cell(i, j);
            map_2d[i][j] = currCell;
        }
    }

    map_2d[GOAL_X][GOAL_Y].isGoal = true;
    map_2d[GOAL_X + 1][GOAL_Y].isGoal = true;
    map_2d[GOAL_X][GOAL_Y + 1].isGoal = true;
    map_2d[GOAL_X + 1][GOAL_Y + 1].isGoal = true;
}

// Draw each point based on colorType
// 0 = Empty
// 1 = Occupied
// 2 = Goal
// 3 = Path
void drawXY2(int x, int y, int colorType) {
    switch(colorType) {
        case 0:
            gfx_color(0, 255, 0);
            break;
        case 1:
            gfx_color(255, 0, 0);
            break;
        case 2:
            gfx_color(255, 223, 0);
            break;
        case 3:
            gfx_color(255, 165, 0);
            break;
        case 4:
            gfx_color(0, 0, 0);
            break;
    }

    gfx_point(x, y);
}


void draw2DMap() {

    gfx_clear();

    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            Cell currCell = map_2d[i][j];

            if (currCell.isGoal) {
                drawXY2(i, j, 2);
                continue;
            }

            if (currCell.hits < freeMin) {
                drawXY2(i, j, 0);
            }

            if (currCell.hits > occupiedMin) {
                drawXY2(i, j, 1);
                continue;
            }
        }
    }

    // Print the current path
    if (path.size() > 0) {
        for (int i = 0; i < path.size(); i++) {
            drawXY2(path[i].x, path[i].y, 3);
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

// Returns the angle based on robot bearing
float angleToCoords(int originalX, int originalY, int targetX, int targetY) {
    int dx = targetX - originalX;
    int dy = targetY - originalY;

    if (dx == 0 && dy < 0) {
        cout << "Flag1" << endl;
        return M_PI / 2;
    }
    if (dx == 0 && dy > 0) {
        cout << "Flag2" << endl;
        return - M_PI / 2;
    }

    if (dx < 0 && dy == 0) {
        cout << "Flag3" << endl;
        return -M_PI + 0.1; // - M_PI will flip a trigger later down
    }

    if (dx > 0 && dy == 0) {
        cout << "Flag4" << endl;
        return 0.0;
    }

    if (dx == 0 && dy == 0) {
        cout << "Flag5" << endl;
        return 0.0;
    }

    double ang = dy / dx;

    // Quadrant 1
    if (0 < dx && 0 < dy) {
        cout << "Flag6" << endl;
        return atan(ang) + (-M_PI / 2);
    }

    // Quadrant 2
    if (0 < dx && 0 > dy) {
        cout << "Flag7" << endl;
        return - atan(ang);
    }

    // Quadrant 3
    if (0 > dx && 0 > dy) {
        cout << "Flag8" << endl;
        return (M_PI / 2) + atan(ang);
    }

    // Quadrant 4
    if (0 > dx && 0 < dy) {
        cout << "Flag9" << endl;
        return atan(ang);
    }

    // // Quadrant 1 (0 - 90)
    // if (0 < dx && 0 < dy) {
    //     cout << "Flag6" << endl;
    //     return - atan(ang);
    // }

    // // Quadrant 2 (90 - 180)
    // if (0 < dx && 0 > dy) {
    //     cout << "Flag7" << endl;
    //     return atan(ang) + (-M_PI / 2);
    // }

    // // Quadrant 3 (180 - 270)
    // if (0 > dx && 0 > dy) {
    //     cout << "Flag8" << endl;
    //     return M_PI + atan(ang);
    // }

    // // Quadrant 4 (270 - 360)
    // if (0 > dx && 0 < dy) {
    //     cout << "Flag9" << endl;
    //     return - atan(ang);
    // }

    // Should never reach here
    cout << "SHOULD NEVER REACH HERE" << endl;
    return 0;
}

void wallFollow(Robot* robot) {

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
        spd = 4;
        trn = 0;
    }


    robot->set_vel(spd + trn, spd - trn);
}

void findClosestIndexInPath(int ox, int oy) {

    int cx = 99999;
    int cy = 99999;
    int smallest = 4;

    int i = 0;

    if (path.size() > 4) {
        i = 4;
    }

    for (i; i < path.size(); i++) {
        Cell currCell = path[i];
        if (abs(currCell.x - ox) + abs(currCell.y - oy) < abs(cx - ox) + abs(cy - oy)) {
            smallest = i;
            cx = currCell.x;
            cy = currCell.y;
        }
        // if (abs(path[i].x - ox) < 3 && abs(path[i].y - oy) < 3) {
        //     currentPathIndex = i;
        // }
    }

    currentPathIndex = smallest;
}

/**
 * Computes angle difference, accounting for the weirdness with the angle
 * system.
 *
 * Positive means turn CCW, negative means turn CW.
 * 
 * Taken from piazaa by Trey Del Bonis
 */
float angle_diff(float from, float to) {
    float from_adj = from < 0 ? (2 * M_PI) + from : from;
    float to_adj = to < 0 ? (2 * M_PI) + to : to;
    float raw_delta = fmodf(to_adj - from_adj, 2 * M_PI);

    if (raw_delta < -M_PI) {
        return (2 * M_PI) + raw_delta;
    }

    if (raw_delta > M_PI) {
        return (-2 * M_PI) + raw_delta;
    }

    return raw_delta;
}

void
callback(Robot* robot)
{
    tick_count += 1;

    if (robot->at_goal()) {
        robot->done();
        return;
    }

    if (tick_count > 20) {
        tick_count = 0;
        draw2DMap();
    }

    cout << "\n===" << endl;

    cout << "tick: " << tick_global_count << " pathIndex: " << currentPathIndex << endl;
    tick_global_count++;
    tick_path_count++;

    int robo_x_in_2dMap = (grid_size / 2) + ceil(robot->pos_x * 4);
    int robo_y_in_2dMap = (grid_size / 2) - ceil(robot->pos_y * 4);

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

            // cout << "A: " << hit.angle << " A,Bearing: " << bearing_from_heading << " hr: " << hit.range
            // << " x,y,t: " << robo_x_in_2dMap << "," << robo_y_in_2dMap << "," << robot->pos_t
            // << " dx,dy: " << x_in_2dMap << ","  << y_in_2dMap << endl;
        }

        // cout << hit.range << "@" << hit.angle << endl;
    }

    if (robot->ranges.size() < 5) {
        return;
    }

    // If there is no path do nothing and wait
    if (path.size() == 0) {
        pathMaze = true;
        return;
    }

    // Set path index to the closest path
    findClosestIndexInPath(robo_x_in_2dMap, robo_y_in_2dMap);

    // If Path is blocked follow the right wall until we figure something out?
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);

    if (fwd > 3 && currentlyPathing) {
        return;
    }

    // If the forward sensor hits wall then path list is incorrect.
    // Map out the maze until new path can be created
    // MAY NEED TO CHANGE LOGIC SO THAT WE DONT FLAG THE FWD < 1.2 before FACING PATH
    if (fwd < 0.6 || tick_wallFollow_count > 0) {

        // cout << "WallFollowTicks: " << tick_wallFollow_count << " isPathing?: " << currentlyPathing << endl;

        int ratio_of_ticks_to_wait = path.size() * 8;

        if (ratio_of_ticks_to_wait < 200) {
            ratio_of_ticks_to_wait = 200;
        }
        cout << "WallFollowTickRatio: " << ratio_of_ticks_to_wait << endl;

        // We want to follow the wall for 90 ticks then pathMaza
        if (tick_wallFollow_count > ratio_of_ticks_to_wait) {
            tick_wallFollow_count = 0;
            return;
        }
        
        pathMaze = true;
        wallFollow(robot);
        tick_wallFollow_count++;
        return;
    }

    // Need to determine which point im heading to?
    Cell tempCell = path[currentPathIndex];

        // If we are at current cell (or close enough) then work on next cell;
    if (abs(tempCell.x - robo_x_in_2dMap) < 4 && abs(tempCell.y - robo_y_in_2dMap) < 4) {
        currentPathIndex += 4;
        tempCell = path[currentPathIndex];
    }


    // Get ArcTan
    float targetTheta = angleToCoords(robo_x_in_2dMap, robo_y_in_2dMap, tempCell.x, tempCell.y);

    if (targetTheta > M_PI) {
        targetTheta = targetTheta - M_PI;
    }

    if (targetTheta < -M_PI) {
        targetTheta = targetTheta + M_PI;
    }

    cout << "x,y,tx,ty = " << robo_x_in_2dMap << "," << robo_y_in_2dMap << "," << tempCell.x << "," << tempCell.y << endl;
    cout << "t, tTheta, d180: " << robot->pos_t << "," << targetTheta << endl;

    // if (abs(tempCell.x - robo_x_in_2dMap) == 0 && abs(tempCell.y - robo_y_in_2dMap) == 0) {
    //     currentPathIndex++;
    //     return;
    // }
    
    // If we are at the correct angle go forwards
    if (abs(targetTheta - robot->pos_t) < 0.1) {
        robot->set_vel(4, 4);
    } else {
        float diff = angle_diff(robot->pos_t, targetTheta) * 1.5;

        if (abs(diff) < 1) {
            diff = diff > 0 ? 2: -2;
        }

        robot->set_vel(-diff, diff);

        cout << "Making turn. diff, targetTheta, rPosT: " << diff << "," << targetTheta << "," << robot->pos_t << endl; 
    }
}

void clearPath() {
    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            map_2d[i][j].isPath = false;
        }
    }
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

void findPathThread(Robot* robot) {
    while(1) {
        if (path.size() > 0 && path[4].x == 154 && path[4].y == 149) { // Remove initial path
            path.clear();
            continue;
        }
        if (pathMaze || initialPath || tick_global_count < 10) {
            cout << "FINDING PATH" << endl;
            currentlyPathing = true;
            tick_path_count = 0;
            
            int robo_x_in_2dMap = (grid_size / 2) + ceil(robot->pos_x * 4);
            int robo_y_in_2dMap = (grid_size / 2) - ceil(robot->pos_y * 4);

            std::vector<Cell> foundPath = aStarAlgo(robo_x_in_2dMap, robo_y_in_2dMap, GOAL_X, GOAL_Y);

            path.clear();

            std::reverse(foundPath.begin(), foundPath.end());

            cout << "FOUND PATH" << endl;

            path = foundPath;
            currentPathIndex = 4;
            pathMaze = false;
            currentlyPathing = false;
            initialPath = false;
        }

    }    
}

// Smooth out the maze
void lineSmoothAlgo() {

    for (int i = 2; i < grid_size-2; i++) {
        for (int j = 2; j < grid_size-2; j++) {
            // Check agencies
            Cell currCell = map_2d[i][j];

            if (currCell.hits > occupiedMin) {

                // North
                if (map_2d[i + 2][j].hits > occupiedMin && map_2d[i + 1][j].hits < occupiedMin) {
                    map_2d[i + 1][j].hits += 2;
                }

                // East
                if (map_2d[i][j + 2].hits > occupiedMin && map_2d[i][j + 1].hits < occupiedMin) {
                    map_2d[i][j + 1].hits += 2;
                }

                // South
                if (map_2d[i - 2][j].hits > occupiedMin && map_2d[i - 1][j].hits < occupiedMin) {
                    map_2d[i - 1][j].hits += 2;
                }

                // West
                if (map_2d[i][j - 2].hits > occupiedMin && map_2d[i][j - 1].hits < occupiedMin) {
                    map_2d[i][j - 1].hits += 2;
                }

            }
        }
    }

}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;

    initilize_2dMap();

    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread pthr(findPathThread, &robot);
    std::thread lsthr(lineSmoothAlgo);

    createWindow();

    return 0;

    // return viz_run(argc, argv);
}
