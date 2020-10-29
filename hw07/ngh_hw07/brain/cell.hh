#ifndef CELL_H
#define CELL_H

class Cell {
    public:
    float hits = 0;
    float misses = 0;
    int x;
    int y;
    bool isGoal = false;
    bool isPath = false;
    double Gscore = 0;
    double Hscore = 0;
    double FScore = 0;
    Cell *parent = nullptr;

    Cell() {
        hits = 0;
        misses = 0;
    }

    Cell(int givenX, int givenY) {
        hits = 0;
        misses = 0;
        x = givenX;
        y = givenY;
    }

    Cell(int givenX, int givenY, Cell *givenParent) {
        hits = 0;
        misses = 0;
        x = givenX;
        y = givenY;
        parent = givenParent;
    }

    ~Cell() {}

    private:
};

#endif