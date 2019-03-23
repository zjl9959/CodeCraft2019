#include "Visualization.h"

#include <queue>

using namespace std;

namespace codecraft2019 {

void Visualization::draw(std::string out_path) {
    ofs.open(out_path);
    if (!ofs.is_open())
        return;
    ofs << "<!DOCTYPE html>"
        << "<head>"
        << "<meta charset='utf-8'>"
        << "<title>CodeCraft2019_TimeSlice</title>"
        << "</head>" << endl
        << "<html>"
        << "<body>" << endl;
    // add code...
    add_script();
    ofs << "</body>"
        << "</html>";
    ofs.close();
}

void Visualization::draw_time_slice() {
    ofs << "<svg id='svg" << time
        << "' width='" << width
        << "' height='" << height
        << "' class='display:none;'>" << endl;
    // add code...
    ofs << "</svg>" << endl;
}

void Visualization::draw_cross(ID id) {

}

void Visualization::draw_road(ID id) {

}

void Visualization::draw_car(ID id) {

}

void Visualization::draw_Id(Coord pos, ID id, int size, bool rotate) {

}

void Visualization::add_script() {
    ofs << "<script>" << endl;
    // add code...
    ofs << "</script>" << endl;
}

void Visualization::init_cross_pos() {
    vector<ID> first_row, first_column;
    vector<bool> seen(ins->raw_crosses.size(), false);
    cross_pos.resize(ins->raw_crosses.size());
    queue<ID> q;
    if (ins->raw_crosses.empty())
        return;
    // 从第一个点开始广搜，第一个点坐标为(0,0)
    q.push(ins->raw_crosses[0].id);
    seen[ins->raw_crosses[0].id];
    cross_pos[ins->raw_crosses[0].id].x = 0;
    cross_pos[ins->raw_crosses[0].id].y = 0;
    int min_posx = INT_MAX, max_posx = INT_MIN, min_posy = INT_MAX, max_posy = INT_MIN;
    while (!q.empty()) {
        ID cross = q.front();
        q.pop();
        if (min_posx > cross_pos[cross].x)
            min_posx = cross_pos[cross].x;
        if (min_posy > cross_pos[cross].y)
            min_posy = cross_pos[cross].y;
        if (max_posx < cross_pos[cross].x)
            max_posx = cross_pos[cross].x;
        if (max_posx < cross_pos[cross].y)
            max_posx = cross_pos[cross].y;
        if (seen[cross])continue;
        ID north_road = ins->raw_crosses[cross].north;
        if (north_road != -1) {
            ID north_cross = ins->raw_roads[north_road].from == cross ?
                ins->raw_roads[north_road].to : ins->raw_roads[north_road].from;
            cross_pos[north_cross].x = cross_pos[cross].x;
            cross_pos[north_cross].y = cross_pos[cross].y - 1;
            seen[north_cross] = true;
            q.push(north_cross);
        }
        ID east_road = ins->raw_crosses[cross].east;
        if (east_road != -1) {
            ID east_cross = ins->raw_roads[east_road].from == cross ?
                ins->raw_roads[east_road].to : ins->raw_roads[east_road].from;
            cross_pos[east_cross].x = cross_pos[cross].x + 1;
            cross_pos[east_cross].y = cross_pos[cross].y;
            seen[east_cross] = true;
            q.push(east_cross);
        }
        ID south_road = ins->raw_crosses[cross].north;
        if (south_road != -1) {
            ID south_cross = ins->raw_roads[south_road].from == cross ?
                ins->raw_roads[south_road].to : ins->raw_roads[south_road].from;
            cross_pos[south_cross].x = cross_pos[cross].x;
            cross_pos[south_cross].y = cross_pos[cross].y + 1;
            seen[south_cross] = true;
            q.push(south_cross);
        }
        ID west_road = ins->raw_crosses[cross].north;
        if (west_road != -1) {
            ID west_cross = ins->raw_roads[west_road].from == cross ?
                ins->raw_roads[west_road].to : ins->raw_roads[west_road].from;
            cross_pos[west_cross].x = cross_pos[cross].x - 1;
            cross_pos[west_cross].y = cross_pos[cross].y;
            seen[west_cross] = true;
            q.push(west_cross);
        }
    }
    // 将cross_pos中的相对坐标转换为其在svg中的坐标。
    int side_with = width * 0.02, side_height = height * 0.02;     // svg中留边大小
    for (int i = 0; i < cross_pos.size(); ++i) {
        cross_pos[i].x = (cross_pos[i].x - min_posx)*width / (max_posx - min_posx) + side_with;
        cross_pos[i].y = (cross_pos[i].y - min_posy)*height / (max_posy - min_posy) + side_height;
    }
}

}
