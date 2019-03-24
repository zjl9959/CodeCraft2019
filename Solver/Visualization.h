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
#define COLOR_TEXT "black"

struct Coord {
    int x;
    int y;
    Coord() {};
    Coord(int X, int Y) :x(X), y(Y) {};
};

class Visualization {
public:
    static constexpr double scale = 1.0;
    static constexpr int width = 1200 * scale;
    static constexpr int height = 1000 * scale;
    static constexpr int cross_size = 50 * scale;
public:
    Visualization(const Instance *instance, const TimeSlice *time_slice) :
        ins(instance), slice(time_slice) {};
    void draw(std::string out_path);
    ~Visualization() { ins = nullptr; };
protected:
    void draw_time_slice(Time time);
    void draw_road(ID id, Time time);
    void draw_Id(int x, int y, ID id, int size, bool rotate);
    void draw_rectangle(int x, int y, int w, int h, std::string fill);
    void draw_line(int x1, int y1, int x2, int y2);
    void add_script();
    void add_css();
    void init_cross_pos();
private:
    const Instance *ins;
    const TimeSlice *slice;  // 时间切片
    std::ofstream ofs;
    std::vector<Coord> cross_pos;   // 每个道路的相对坐标
};

}

#endif // !ROADEF_2019_VISUALIZATION_H
