
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

Cell* findCell(std::list<Cell> givenloc, int givX, int givY) {
    std::list<Cell>::iterator it;
    for (it = givenloc.begin(); it != givenloc.end(); it++) {
        if (it->x == givX && it->y == givY) {
            return &*it;
        }
    }

    return nullptr;
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

std::vector<Cell> aStarAlgo(int initialX, int initialY, int goalX, int goalY) {
    std::list<Cell*> openCell, closeCell;

    openCell.push_back(new Cell(initialX, initialY));

    Cell *current = nullptr;

    cout << "Beginning A* Algo" << endl;

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

    cout << "ENDING A* Algo. PATH LENGTH: " << result.size() << endl;

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

void
callback(Robot* robot)
{
    tick_count += 1;

    if (tick_count > 20) {
        tick_count = 0;
        draw2DMap();
    }

    cout << "\n===" << endl;

    cout << "tick: " << tick_global_count << " tickPathCount: " << tick_path_count << endl;
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

    cout << "x,y,t = "
         << robo_x_in_2dMap << ","
         << robo_y_in_2dMap << endl;
    // robot->set_vel(robot->pos_t, -robot->pos_t);
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
        if (tick_path_count > 30) {
            tick_path_count = 0;

            cout << "Updating Algo" << endl;
            
            int robo_x_in_2dMap = (grid_size / 2) + ceil(robot->pos_x * 4);
            int robo_y_in_2dMap = (grid_size / 2) - ceil(robot->pos_y * 4);

            std::vector<Cell> foundPath = aStarAlgo(robo_x_in_2dMap, robo_y_in_2dMap, GOAL_X, GOAL_Y);

            path.clear();

            path = foundPath;
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

    createWindow();

    return 0;

    // return viz_run(argc, argv);
}
