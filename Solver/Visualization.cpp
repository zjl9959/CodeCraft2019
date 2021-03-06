#include "Visualization.h"

#include <queue>

using namespace std;

namespace codecraft2019 {

void Visualization::draw(std::string out_path) {
    init_cross_pos();
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
    int font_size = 30;
    ofs << "<div style='font-size:" << font_size * scale << "px;'>" << endl;
    ofs << "<br><text>Current Time:</text>" << endl;
    ofs << "<text id='text_time'> 0 </text>" << endl;
    ofs << "<button onclick='prev()' style='font-size:"
        << font_size * scale << "px;'>" << "<-" << "</button>" << endl;
    ofs << "<button onclick='next()' style='font-size:"
        << font_size * scale << "px;'>" << "->" << "</button>" << endl;
    ofs << "</div>" << endl;
    add_css();
    int time = 0;
    for (auto it = slice->cbegin(); it != slice->cend(); ++it) {
        draw_time_slice(time);
        ++time;
    }
    add_script();
    ofs << "</body>"
        << "</html>";
    ofs.close();
}

// 对每一个时间切片，在svg上绘制一幅图
void Visualization::draw_time_slice(Time time) {
    ofs << "<svg id='svg" << time
        << "' width='" << width
        << "' height='" << height;
    if (time == 0)
        ofs << "' style='display:inline;'>" << endl;
    else
        ofs << "' style='display:none;'>" << endl;
    // 画出每一条车道
    for (int i = 0; i < slice->at(time).size(); ++i) {
        draw_road(i, time);
    }
    // 为每个路口添加id
    int text_size = 10;
    for (int i = 0; i < cross_pos.size(); ++i) {
        draw_Id(cross_pos[i].x - text_size, cross_pos[i].y + text_size/2, i, text_size, 0);
    }
    ofs << "</svg>" << endl;
}

// 画出车道，路上的车，以及ID信息
void Visualization::draw_road(ID id, Time time) {
    bool duplex = ins->raw_roads[id].is_duplex;
    Channel channel = ins->raw_roads[id].channel;
    ID from = ins->raw_roads[id].from;
    ID to = ins->raw_roads[id].to;
    Length len = ins->raw_roads[id].length;
    int channel_width = cross_size / (channel*(duplex + 1)); // 每条车道的宽度
    bool rotate = (cross_pos[from].x == cross_pos[to].x);
    int basic_x, basic_y, delt_x, delt_y;   // basic_x,baisc_y基准点坐标；delt_x,delt_y车道宽度；
    if (rotate) {
        if (cross_pos[to].y > cross_pos[from].y) {          // 从上到下
            basic_x = cross_pos[from].x - cross_size / 2;
            basic_y = cross_pos[from].y + cross_size / 2;
            delt_x = channel_width;
            delt_y = cross_pos[to].y - cross_pos[from].y - cross_size;
        } else {                                            // 从下到上
            basic_x = cross_pos[from].x + cross_size / 2;
            basic_y = cross_pos[from].y - cross_size / 2;
            delt_x = -channel_width;
            delt_y = cross_pos[to].y - cross_pos[from].y + cross_size;
        }
    } else {
        if (cross_pos[to].x > cross_pos[from].x) {          // 从左到右
            basic_x = cross_pos[from].x + cross_size / 2;
            basic_y = cross_pos[from].y + cross_size / 2;
            delt_y = -channel_width;
            delt_x = cross_pos[to].x - cross_pos[from].x - cross_size;
        } else {                                            // 从右到左
            basic_x = cross_pos[from].x - cross_size / 2;
            basic_y = cross_pos[from].y - cross_size / 2;
            delt_y = channel_width;
            delt_x = cross_pos[to].x - cross_pos[from].x + cross_size;
        }
    }
    int ccount = 0;         // channel count, 代表当前是第几个车道
    for (int i = 0; i <= duplex; ++i) {
        for (int j = 0; j < channel; ++j) {
            if (rotate)                         // 画车道
                draw_rectangle(basic_x + ccount*delt_x, basic_y, delt_x, delt_y, COLOR_ROAD);
            else
                draw_rectangle(basic_x, basic_y + ccount*delt_y, delt_x, delt_y, COLOR_ROAD);
            /* 太占输出文件内存
            for (int l = 0; l < len; ++l) {     // 画道路长度刻度线
                if (rotate)
                    draw_line(basic_x, basic_y + l * delt_y / len, basic_x + delt_x * channel*(duplex + 1), basic_y + l * delt_y / len);
                else
                    draw_line(basic_x + l * delt_x / len, basic_y, basic_x + l * delt_x / len, basic_y + delt_y * channel*(duplex + 1));
            }
            */
            if (i == 0) {
                int font_size = 10;
                if (rotate)                         // 绘制道路id
                    draw_Id(basic_x + (delt_x > 0 ? -font_size : font_size), basic_y + delt_y / 2, id, font_size, 0);
                else
                    draw_Id(basic_x + delt_x / 2, basic_y + (delt_y > 0 ? -font_size : font_size), id, font_size, 0);
            }
            ++ccount;
        }
    }
    if (duplex) {   // 道路中间分割线
        if (rotate)
            draw_line(basic_x + channel * delt_x, basic_y, basic_x + channel * delt_x, basic_y + delt_y);
        else
            draw_line(basic_x, basic_y + channel * delt_y, basic_x + delt_x, basic_y + channel * delt_y);
    }
    // 画出来此条道路上的每一辆车
    for (auto it = slice->at(time)[id].begin(); it != slice->at(time)[id].end(); ++it) {
        if (rotate) {
            Length pos = it->pos;
            if (it->channel >= channel)    // 说明车在反向车道上
                pos = len - pos + 1;
            draw_rectangle(               // 画车
                basic_x + delt_x * it->channel,
                basic_y + (pos - 1) * delt_y/len,
                delt_x,
                delt_y/len,
                it->state ? COLOR_CAR_STOP : COLOR_CAR_WAIT);
            draw_Id(                      // 画车的id
                basic_x + delt_x * (it->channel + (delt_x > 0 ? 0 : 1)),
                basic_y + ((pos - 1) + (delt_y > 0 ? 1 : 0)) * delt_y/len - 1,
                it->car_id,
                (abs(delt_y) * 4) / (len * 5), 0);
        } else {
            Length pos = it->pos;
            if (it->channel >= channel)
                pos = len - pos + 1;
            draw_rectangle(
                basic_x + (pos - 1)*delt_x / len,
                basic_y + delt_y * it->channel,
                delt_x / len,
                delt_y,
                it->state ? COLOR_CAR_STOP : COLOR_CAR_WAIT);
            draw_Id(
                basic_x + (pos + (delt_x > 0 ? 0 : 1))*delt_x / len,
                basic_y + delt_y * (it->channel + (delt_y > 0 ? 0 : 1)) + 1,
                it->car_id,
                (abs(delt_x) * 4) / (len * 5), 1);
        }
    }
}

// 如果不旋转，给出文本位于的矩形块左下角的坐标；如果旋转，给出右上角的坐标
void Visualization::draw_Id(int x, int y, ID id, int size, bool rotate) {
    if (size > 10)
        size = 10;
    if (rotate)
        y += size;
    ofs << "<text x='" << x
        << "' y='" << y
        << "' font-size='" << size;
    if (rotate) {
        ofs << "' transform='rotate(90 "
            << x << " " << y - size << ")";
    }
    ofs << "'>" << id << "</text>" << endl;
}

// 在svg中绘制一个矩形
void Visualization::draw_rectangle(int x, int y, int w, int h, std::string fill) {
    if (w < 0) {
        x += w;
        w = -w;
    }
    if (h < 0) {
        y += h;
        h = -h;
    }
    ofs << "<rect x='" << x
        << "' y='" << y
        << "' width='" << w
        << "' height='" << h
        << "' fill='" << fill
        << "'/>" << endl;
}

// 画一条白色的线，用于标记车道刻度
void Visualization::draw_line(int x1, int y1, int x2, int y2) {
    ofs << "<line x1='" << x1
        << "' y1='" << y1
        << "' x2='" << x2
        << "' y2='" << y2
        << "'/>" << endl;
}

// 一次只显示一个时间片的SVG,prev()函数表示上一个，next()函数表示下一个
void Visualization::add_script() {
    ofs << "<script>" << endl;
    // 定义变量，表示当前显示的时间片
    ofs << "var time=0;" << endl;
    ofs << "document.onkeydown = function(e) {" << endl
        << "if ((e || event).keyCode == 68)" << endl
        << "    next();" << endl
        << "if ((e || event).keyCode == 65)" << endl
        << "    prev();}" << endl;
    // 定义prev函数
    ofs << "function prev() {" << endl
        << "    if(time==0)return;" << endl
        << "    doc1 = document.getElementById('svg'+time.toString());" << endl
        << "    if(doc1!=null)doc1.style.display='none';" << endl
        << "    time=time-1;" << endl
        << "    doc2 = document.getElementById('svg'+time.toString());" << endl
        << "    if(doc2!=null)doc2.style.display='inline';" << endl
        << "    document.getElementById('text_time').innerHTML=time.toString();" << endl
        << "}" << endl;
    // 定义next函数
    ofs << "function next() {" << endl
        << "    doc1 = document.getElementById('svg'+time.toString());" << endl
        << "    if(doc1!=null)doc1.style.display='none';" << endl
        << "    time=time+1;" << endl
        << "    doc2 = document.getElementById('svg'+time.toString());" << endl
        << "    if(doc2!=null)doc2.style.display='inline';" << endl
        << "    document.getElementById('text_time').innerHTML=time.toString();" << endl
        << "}" << endl;
    ofs << "</script>" << endl;
}

void Visualization::add_css() {
    ofs << "<style>" << endl;
    // 白色，1px粗细线条
    ofs << "line {"
        << "stroke:white;"
        << "stroke-width:1;"
        << "}" << endl;
    // 矩形，轮廓宽为1，颜色为黑色
    ofs << "rect {"
        << "stroke-width:1;"
        << "stroke:black;"
        << "}" << endl;
    // 黑色文本
    ofs << "text {"
        << "fill:" << COLOR_TEXT << ";"
        << "}" << endl;
    ofs << "</style>" << endl;
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
        if (max_posy < cross_pos[cross].y)
            max_posy = cross_pos[cross].y;
        if (seen[cross])continue;
        seen[cross] = true;
        ID north_road = ins->raw_crosses[cross].north;
        if (north_road != -1) {
            ID north_cross = ins->raw_roads[north_road].from == cross ?
                ins->raw_roads[north_road].to : ins->raw_roads[north_road].from;
            cross_pos[north_cross].x = cross_pos[cross].x;
            cross_pos[north_cross].y = cross_pos[cross].y - 1;
            q.push(north_cross);
        }
        ID east_road = ins->raw_crosses[cross].east;
        if (east_road != -1) {
            ID east_cross = ins->raw_roads[east_road].from == cross ?
                ins->raw_roads[east_road].to : ins->raw_roads[east_road].from;
            cross_pos[east_cross].x = cross_pos[cross].x + 1;
            cross_pos[east_cross].y = cross_pos[cross].y;
            q.push(east_cross);
        }
        ID south_road = ins->raw_crosses[cross].south;
        if (south_road != -1) {
            ID south_cross = ins->raw_roads[south_road].from == cross ?
                ins->raw_roads[south_road].to : ins->raw_roads[south_road].from;
            cross_pos[south_cross].x = cross_pos[cross].x;
            cross_pos[south_cross].y = cross_pos[cross].y + 1;
            q.push(south_cross);
        }
        ID west_road = ins->raw_crosses[cross].west;
        if (west_road != -1) {
            ID west_cross = ins->raw_roads[west_road].from == cross ?
                ins->raw_roads[west_road].to : ins->raw_roads[west_road].from;
            cross_pos[west_cross].x = cross_pos[cross].x - 1;
            cross_pos[west_cross].y = cross_pos[cross].y;
            q.push(west_cross);
        }
    }
    // 将cross_pos中的相对坐标转换为其在svg中的坐标。
    int side_with = width * 0.04, side_height = height * 0.04;     // svg中留边大小
    for (int i = 0; i < cross_pos.size(); ++i) {
        cross_pos[i].x = (cross_pos[i].x - min_posx)*(width - 2 * side_with) /
            (max_posx - min_posx) + side_with;
        cross_pos[i].y = (cross_pos[i].y - min_posy)*(height - 2*side_height) /
            (max_posy - min_posy) + side_height;
    }
}

}
