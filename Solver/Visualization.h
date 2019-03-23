#pragma once
#ifndef ROADEF_2019_VISUALIZATION_H
#define ROADEF_2019_VISUALIZATION_H
#include <fstream>
#include <string>
#include <vector>

#include "Instance.h"

namespace codecraft2019 {

#define COLOR_ROAD "gray"
#define COLOR_CROSS "yellow"
#define COLOR_CAR_WAIT "green"
#define COLOR_CAR_STOP "red"
#define COLOR_TEXT "white"

struct Coord {
    int x;
    int y;
    Coord() {};
    Coord(int X, int Y) :x(X), y(Y) {};
};

class Visualization {
public:
    static constexpr int width = 1000;
    static constexpr int height = 1000;
    static constexpr int slice_interval = 100;
public:
    Visualization(Instance *instance) :ins(instance) {};
    void draw(std::string out_path);
    ~Visualization() { ins = nullptr; };
protected:
    void draw_time_slice();
    void draw_cross(ID id);
    void draw_road(ID id);
    void draw_car(ID id);
    void draw_Id(Coord pos, ID id, int size, bool rotate);
    void add_script();
    void init_cross_pos();
private:
    Instance *ins;
    std::ofstream ofs;
    std::vector<Coord> cross_pos;   // 每个道路的相对坐标
    int time = 0;
};

}

#endif // !ROADEF_2019_VISUALIZATION_H
