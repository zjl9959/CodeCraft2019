#pragma once
#ifndef CODE_CRAFT_2019_OUTPUT_H
#define CODE_CRAFT_2019_OUTPUT_H
/*
 * 将Solution转换为官方规定的输出格式。
 * 将结果保存到指定路径的文件中。
 */
#include "Instance.h"

namespace codecraft2019 {

struct Routine {
    ID car_id;
    Time start_time;
    std::vector<ID> roads;
	std::vector<ID> crosses;//由roads转换而来的路口
};

struct Output {
    Output(Instance* instance): ins_(instance) {};
    ~Output() { ins_ = nullptr; }
    bool save(Environment &env);
    Instance* ins_;
    std::vector<Routine> routines;
};

}

#endif // !CODE_CRAFT_2019_OUTPUT_H
