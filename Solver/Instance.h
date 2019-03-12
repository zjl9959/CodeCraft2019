#pragma once
#ifndef CODE_CRAFT_2019_INSTANCE_H
#define CODE_CRAFT_2019_INSTANCE_H
/*
 * 根据描述文档定义算例结构。
 * 从输入文档中加载算例数据。
 */
#include <vector>

#include "Common.h"

namespace codecraft2019 {

struct Road {
    ID id;                  // 道路ID
    Length length;          // 道路长度
    Speed speed_limit;      // 道路限速
    int road_num;           // 每方向车道数目
    ID begin;               // 起点ID
    ID end;                 // 终点ID
    bool bidirection;       // 是否为双向车道
};

struct Cross {
    ID id;                  // 交叉路口ID
    ID north;               // 北/上方路口
    ID east;                // 东/右方路口
    ID west;                // 西/左方路口
    ID south;               // 南/下方路口
};

struct Car {
    ID id;                  // 车辆ID
    ID origin;              // 起点ID
    ID destination;         // 终点ID
    Speed max_speed;        // 最大行驶速度
    Time go_time;           // 计划出发时间
};

struct Instance {
    Instance();
    void load(Environment &env);
    ~Instance();

    std::vector<Road> roads;
    std::vector<Car> cars;
    std::vector<Cross> crosses;
};

}

#endif // !CODE_CRAFT_2019_INSTANCE_H
