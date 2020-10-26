#ifndef CELL_H
#define CELL_H

class Cell {
    public:
    float hits = 0;
    float misses = 0;
    int x;
    int y;

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

    ~Cell() {}

    private:
};

#endif