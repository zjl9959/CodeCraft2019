#pragma once
#ifndef CODE_CRAFT_2019_INSTANCE_H
#define CODE_CRAFT_2019_INSTANCE_H
/*
 * 根据描述文档定义算例结构。
 * 从输入文档中加载算例数据。
 */
#include <vector>

#include "Common.h"
#include "Utility.h"

namespace codecraft2019 {

struct Road {
    ID id;                  // 道路ID
    Length length;          // 道路长度
    Speed speed;            // 道路限速
    Channel channel;        // 每方向车道数目
    ID from;                // 起点ID
    ID to;                  // 终点ID
    bool is_duplex;         // 是否为双向车道
    Road(ID Id, Length Length, Speed Speed, Channel Channel, ID From, ID To, bool Is_duplex) :
        id(Id), length(Length), speed(Speed), channel(Channel), from(From), to(To), is_duplex(Is_duplex) {};
};

struct Car {
    ID id;                  // 车辆ID
    ID from;                // 起点ID
    ID to;                  // 终点ID
    Speed speed;            // 最大行驶速度
    Time plan_time;         // 计划出发时间
    Car(ID Id, ID From, ID To, Speed Speed, Time Plan_time) :
        id(Id), from(From), to(To), speed(Speed), plan_time(Plan_time) {};
};

struct Cross {
    ID id;                  // 交叉路口ID
    ID north;               // 北/上方路口
    ID east;                // 东/右方路口
    ID west;                // 西/左方路口
    ID south;               // 南/下方路口
    Cross(ID Id, ID North, ID East, ID West, ID South) :id(Id), north(North), east(East), south(South) {};
};

struct Instance {
    Instance() {};
    Instance(Environment &env) { load(env); };
    ~Instance() {};

    bool load(Environment &env);    // 从输入路径中加载算例。

    bool valid = true;    // 指示加载的算例是否合法。

    std::vector<Road> roads;
    std::vector<Car> cars;
    std::vector<Cross> crosses;

    ZeroBasedConsecutiveIdMap<ID, ID> road_map;
    ZeroBasedConsecutiveIdMap<ID, ID> car_map;
    ZeroBasedConsecutiveIdMap<ID, ID> cross_map;
};

}

#endif // !CODE_CRAFT_2019_INSTANCE_H
