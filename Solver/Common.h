#pragma once
#ifndef CODE_CRAFT_2019_COMMON_H
#define CODE_CRAFT_2019_COMMON_H

#include <string>

namespace codecraft2019 {

/*******定义基本的数据类型********/
#define INVALID_ID -1           // 无效的ID
#define LENGTH_MAX INT_MAX
#define MAX_TIME INT_MAX
#define MAX_CROSS_ROAD_NUM 4
#define LATEST_PLAN_TIME 11
using ID = int;
using Length = int;
using Speed = int;
using Time = int;
using Channel = int;
enum Turn { Left, Right, Front, InvalidTurn };
enum IDMap{RoadMap = 5000,CrossMap = 1,CarMap = 10000};
enum STATE{STATE_waitRun,STATE_terminated};


/***********定义输入输出路径环境***************/
struct Environment {
    std::string car_path;       // 输入算例car.txt路径。
    std::string road_path;      // 输入算例road.txt路径。
    std::string cross_path;     // 输入算例cross.txt路径。
    std::string answer_path;    // 输出结果answer.txt路径。
    Environment(char* Car_path, char* Road_path, char* Cross_path, char* Answer_path) :
        car_path(Car_path), road_path(Road_path), cross_path(Cross_path), answer_path(Answer_path) {};
};

/**********  定义算法参数配置*****************/
struct Configure {
    // [TODO]增加其它配置参数。
    double maxseconds = 10.0;           // 最大运行时间，单位秒。
    double maxmilliseconds = 10000.0;    // 最大运行时间，单位毫秒。
    size_t maxiter = 1 << 31;           // 最大迭代次数。
};

}

#endif // !CODE_CRAFT_2019_COMMON_H